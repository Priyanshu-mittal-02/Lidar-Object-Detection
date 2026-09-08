import serial
import struct
import numpy as np
import time
import cv2
import open3d as o3d
import threading
import queue
import os
from datetime import datetime
import statistics

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = '/dev/ttyUSB0'  # Change to '/dev/ttyUSB0' or '/dev/ttyACM0' for RPi
BAUD_RATE = 921600

# Sensor Properties
START_ANGLE = -48.0
ANGULAR_RES = 0.3516
SCAN_RATE_HZ = 20.0  # Standard for many LZR sensors, needed for dt
DT = 1.0 / SCAN_RATE_HZ  # Frame interval (~0.05s)

# Calibration / Background Subtraction
CALIBRATION_FRAMES = 3000
GRID_CELL_SIZE = 0.05  # Size of background grid cells (meters)

# Filtering Limits
MIN_RANGE_M = 0.10
MAX_RANGE_M = 2.25
MAX_NEIGHBOR_JUMP = 0.15

# Vehicle Logic
# [CHANGED] These are now initial defaults, but speed will update dynamically
DEFAULT_SPEED_KMPH = 5.0
MIN_SPEED_KMPH = 1.0  # Lower limit to prevent division by zero or infinite stretching
TRIGGER_THRESHOLD = 50  # Minimum points to consider it a vehicle
IDLE_TIMEOUT = 0.8  # Seconds of silence to consider vehicle passed

# --- Image Generation Parameters ---
GRID_RES = 0.005  # 5mm per pixel resolution
MAX_DIST_INTENSITY = 50.0

# 3D Rotation Correction (Degrees)
ROTATION_X_DEG = 20.0  # Pitch
ROTATION_Y_DEG = 0.0  # Roll
ROTATION_Z_DEG = 0.0  # Yaw

# Dynamic Frame Settings
FRAME_PADDING_M = 0.2
MIN_IMAGE_DIM_M = 1.0

processing_queue = queue.Queue()


# =====================================================
# HELPER: ROBUST MEDIAN (MAD)
# =====================================================
def robust_median(distances):
    """Calculates median ignoring outliers using Median Absolute Deviation."""
    if len(distances) < 5:
        return None
    median = statistics.median(distances)
    deviations = [abs(d - median) for d in distances]
    mad = statistics.median(deviations)
    if mad == 0:
        return median
    filtered = [d for d in distances if abs(d - median) <= 3 * mad]
    return statistics.median(filtered) if filtered else None


# =====================================================
# HELPER: SIMPLE TRACKER FOR SPEED ESTIMATION
# =====================================================
class SpeedEstimator:
    def __init__(self):
        self.prev_centroid = None
        self.current_speed_mps = DEFAULT_SPEED_KMPH / 3.6
        # Smoothing factor (alpha). 0.1 = slow adaptation, 0.9 = fast adaptation
        self.alpha = 0.3

    def update(self, current_points_xy):
        """
        Calculates speed based on centroid shift between frames.
        Returns the smoothed speed in m/s.
        """
        if len(current_points_xy) < 10:
            return self.current_speed_mps  # Not enough points to track reliable centroid

        # 1. Calculate Centroid of current frame
        # We only care about X/Y movement.
        # Note: In raw data, X is usually depth/width relative to sensor.
        # We assume the vehicle moves roughly perpendicular to the scan plane or
        # we track the lateral movement if the sensor is side-mounted.
        # For a side-mounted profiler, the object doesn't "move" in X/Y much,
        # it passes THROUGH the beam.
        # However, for speed detection, we usually need 2 consecutive frames
        # where the object has moved.
        # IF THIS IS A 2D PROFILER (Scan line stays fixed, car moves):
        # We cannot detect speed solely from 2D X/Y slice changes unless the
        # car features move (feature tracking) or we have an external trigger.

        # HOWEVER, the user asked to merge the provided code which does:
        # speed = norm(centroid2 - centroid1) / dt
        # This implies the sensor sees the object moving *within* its field of view
        # OR this is a 3D LiDAR.
        # Assuming typical LZR behavior (2D profiler), speed detection purely from
        # one slice is hard. But if the object moves laterally *across* the beam:

        curr_centroid = np.mean(current_points_xy, axis=0)

        if self.prev_centroid is not None:
            # Euclidean distance moved in the 2D plane
            dist = np.linalg.norm(curr_centroid - self.prev_centroid)

            # Instantaneous speed
            inst_speed = dist / DT

            # Sanity check: If speed is insane (> 150km/h) or too low, ignore or clamp
            if inst_speed > (150 / 3.6): inst_speed = self.current_speed_mps

            # Low-pass filter to smooth speed
            self.current_speed_mps = (self.alpha * inst_speed) + ((1 - self.alpha) * self.current_speed_mps)

            # Clamp to minimum to prevent stopping
            if self.current_speed_mps < (MIN_SPEED_KMPH / 3.6):
                self.current_speed_mps = (MIN_SPEED_KMPH / 3.6)

        self.prev_centroid = curr_centroid
        return self.current_speed_mps

    def reset(self):
        self.prev_centroid = None
        self.current_speed_mps = DEFAULT_SPEED_KMPH / 3.6


# =====================================================
# BACKGROUND PROCESSOR (3D ROTATION + DYNAMIC SIZE)
# =====================================================
def background_processor():
    print("🧵 Background Processor Started")
    while True:
        try:
            # We now receive the computed average speed for this vehicle as well
            extracted_stack, ts, avg_speed_mps = processing_queue.get()

            # Safety Check
            if len(extracted_stack) < 30:
                print(f"⚠️ Vehicle {ts} Skipped: Too few points")
                processing_queue.task_done()
                continue

            print(f"⚙️ Processing {ts} | Avg Speed Used: {avg_speed_mps * 3.6:.1f} km/h")

            folder = f"vehicle_{ts}"
            os.makedirs(folder, exist_ok=True)

            # ---------------------------------------------------------
            # 1. Cleaning
            # ---------------------------------------------------------
            pcd = o3d.geometry.PointCloud()
            # The 'extracted_stack' already has Z calculated based on dynamic speed
            pcd.points = o3d.utility.Vector3dVector(np.array(extracted_stack, dtype=np.float64))

            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=0.8)
            if len(pcd.points) == 0:
                processing_queue.task_done()
                continue

            pcd, _ = pcd.remove_radius_outlier(nb_points=12, radius=0.06)
            if len(pcd.points) == 0:
                processing_queue.task_done()
                continue

            # ---------------------------------------------------------
            # 2. Standard Side View Transformation
            # ---------------------------------------------------------
            # R_standard: Old Z(Time) -> New X(Length), Old Y -> New Y, -Old X -> New Z
            R_standard = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
            pts_side = (R_standard @ np.asarray(pcd.points).T).T

            # ---------------------------------------------------------
            # 2.1 Apply Full 3D Mounting Rotation Correction
            # ---------------------------------------------------------
            rad_x = np.radians(ROTATION_X_DEG)
            rad_y = np.radians(ROTATION_Y_DEG)
            rad_z = np.radians(ROTATION_Z_DEG)

            Rx = np.array([[1, 0, 0], [0, np.cos(rad_x), -np.sin(rad_x)], [0, np.sin(rad_x), np.cos(rad_x)]])
            Ry = np.array([[np.cos(rad_y), 0, np.sin(rad_y)], [0, 1, 0], [-np.sin(rad_y), 0, np.cos(rad_y)]])
            Rz = np.array([[np.cos(rad_z), -np.sin(rad_z), 0], [np.sin(rad_z), np.cos(rad_z), 0], [0, 0, 1]])

            R_total = Rz @ Ry @ Rx
            pts_side = (R_total @ pts_side.T).T

            # Save Side View PCD
            pcd_side = o3d.geometry.PointCloud()
            pcd_side.points = o3d.utility.Vector3dVector(pts_side)
            o3d.io.write_point_cloud(f"{folder}/side_view.pcd", pcd_side)

            # ---------------------------------------------------------
            # 3. Dynamic Frame Calculation
            # ---------------------------------------------------------
            min_x, min_y = np.min(pts_side[:, :2], axis=0)
            max_x, max_y = np.max(pts_side[:, :2], axis=0)

            min_x -= FRAME_PADDING_M;
            max_x += FRAME_PADDING_M
            min_y -= FRAME_PADDING_M;
            max_y += FRAME_PADDING_M

            if (max_x - min_x) < MIN_IMAGE_DIM_M: max_x = min_x + MIN_IMAGE_DIM_M
            if (max_y - min_y) < MIN_IMAGE_DIM_M: max_y = min_y + MIN_IMAGE_DIM_M

            width_m = max_x - min_x
            height_m = max_y - min_y

            x_bins = int(width_m / GRID_RES)
            y_bins = int(height_m / GRID_RES)

            # ---------------------------------------------------------
            # 4. Image Generation
            # ---------------------------------------------------------
            bev = np.zeros((y_bins, x_bins), dtype=np.float32)

            xi = ((pts_side[:, 0] - min_x) / GRID_RES).astype(np.int32)
            yi = ((pts_side[:, 1] - min_y) / GRID_RES).astype(np.int32)

            valid_indices = (xi >= 0) & (xi < x_bins) & (yi >= 0) & (yi < y_bins)
            xi = xi[valid_indices]
            yi = yi[valid_indices]
            v_points = pts_side[valid_indices]

            if len(v_points) > 0:
                dist = np.sqrt(v_points[:, 0] ** 2 + v_points[:, 1] ** 2)
                intensity = 1.0 - np.minimum(dist / MAX_DIST_INTENSITY, 1.0)
                bev[yi, xi] = intensity

                bev_img = (bev * 255).astype(np.uint8)
                bev_img = np.flipud(bev_img)
                bev_img = cv2.GaussianBlur(bev_img, (3, 3), 0)

                img_path = f"{folder}/side_view_image.png"
                cv2.imwrite(img_path, bev_img)
                print(f"✅ Vehicle {ts} Processed: {img_path}")
            else:
                print(f"⚠️ Vehicle {ts} Skipped: No points in generated frame")

            processing_queue.task_done()

        except Exception as e:
            print(f"❌ Error in background processor: {e}")
            import traceback
            traceback.print_exc()
            processing_queue.task_done()


# =====================================================
# MAIN SYSTEM
# =====================================================
class TollPlazaSystem:
    def __init__(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            print(f"❌ Serial Error: {e}")
            print("Check connection and permissions (sudo chmod 666 /dev/ttyUSB0)")
            exit(1)

        self.background_matrix = set()
        self.is_stacking = False
        self.current_extracted = []
        self.last_detection_time = time.time()
        self.start_capture_time = None

        # [NEW] Speed Estimation
        self.speed_estimator = SpeedEstimator()
        self.vehicle_speeds = []  # To store speed of every frame to average later
        self.last_z = 0.0  # Keep track of Z position integration

    def calibrate(self):
        print(f"⌛ Robust Zero Plane Training ({CALIBRATION_FRAMES} frames)...")
        os.makedirs("calibration", exist_ok=True)
        beam_history = {}
        frames = 0

        while frames < CALIBRATION_FRAMES:
            if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
                size_bytes = self.ser.read(2)
                if len(size_bytes) < 2: continue
                size = struct.unpack('<H', size_bytes)[0]
                body = self.ser.read(size)
                if len(body) < size: continue
                self.ser.read(2)  # Checksum

                if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                    data = body[3:]
                    for i in range(len(data) // 2):
                        d = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                        if d > 0:
                            beam_history.setdefault(i, []).append(d)
                    frames += 1
                    if frames % 500 == 0: print(f"    Frames: {frames}/{CALIBRATION_FRAMES}")

        clean_zero_pts = []
        idxs = sorted(beam_history.keys())
        for j, idx in enumerate(idxs):
            robust_mm = robust_median(beam_history[idx])
            if robust_mm is None: continue

            dist_m = robust_mm / 1000.0
            if not (MIN_RANGE_M <= dist_m <= MAX_RANGE_M): continue

            angle = np.radians(START_ANGLE + idx * ANGULAR_RES)
            x, y = dist_m * np.cos(angle), dist_m * np.sin(angle)
            clean_zero_pts.append((x, y, 0.0))

            ix, iy = int(np.floor(x / GRID_CELL_SIZE)), int(np.floor(y / GRID_CELL_SIZE))
            self.background_matrix.add((ix, iy))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(clean_zero_pts))
        o3d.io.write_point_cloud("calibration/lzr_zero_plane.pcd", pcd)
        print(f"✅ Robust Zero Plane Saved ({len(clean_zero_pts)} points)")

    def get_raw_frame(self):
        """Reads one full scan from the LiDAR."""
        if self.ser.read(1) == b'\xfc' and self.ser.read(3) == b'\xfd\xfe\xff':
            size_bytes = self.ser.read(2)
            if len(size_bytes) < 2: return None
            size = struct.unpack('<H', size_bytes)[0]
            body = self.ser.read(size)
            if len(body) < size: return None
            self.ser.read(2)  # Checksum

            if len(body) > 2 and struct.unpack('<H', body[:2])[0] == 50011:
                data = body[3:]
                pts = []
                for i in range(len(data) // 2):
                    d_mm = struct.unpack('<H', data[i * 2:i * 2 + 2])[0]
                    if 100 < d_mm < 3500:
                        dist = d_mm / 1000.0
                        ang = np.radians(START_ANGLE + i * ANGULAR_RES)
                        pts.append([dist * np.cos(ang), dist * np.sin(ang)])
                return pts
        return None

    def run_forever(self):
        t_thread = threading.Thread(target=background_processor, daemon=True)
        t_thread.start()

        self.calibrate()
        print("🚀 System Live - Waiting for vehicles...")

        while True:
            raw_xy = self.get_raw_frame()
            if not raw_xy: continue

            # Background Subtraction
            extracted_xy = []
            for p in raw_xy:
                ix, iy = int(np.floor(p[0] / GRID_CELL_SIZE)), int(np.floor(p[1] / GRID_CELL_SIZE))
                if (ix, iy) not in self.background_matrix:
                    extracted_xy.append(p)

            # [NEW] Dynamic Speed Calculation
            # We estimate speed based on the currently extracted slice
            # Note: This is a simplified estimation. If extracted_xy is empty, speed holds previous value.
            current_speed = self.speed_estimator.update(np.array(extracted_xy) if len(extracted_xy) > 0 else [])

            # Trigger Logic
            if len(extracted_xy) > TRIGGER_THRESHOLD:
                if not self.is_stacking:
                    self.is_stacking = True
                    self.start_capture_time = time.time()
                    self.current_extracted = []
                    self.vehicle_speeds = []
                    self.last_z = 0.0  # Reset Z integration
                    self.speed_estimator.reset()  # Reset tracker
                    print("🚗 Vehicle Detected")

                self.last_detection_time = time.time()
                self.vehicle_speeds.append(current_speed)

                # [NEW] Integrate Z based on instantaneous speed
                # Z = Z_prev + (speed * dt)
                step_z = current_speed * DT
                current_z_pos = self.last_z + step_z

                # Add points with the calculated Z
                for p in extracted_xy:
                    self.current_extracted.append([p[0], p[1], current_z_pos])

                self.last_z = current_z_pos

            elif self.is_stacking and (time.time() - self.last_detection_time > IDLE_TIMEOUT):
                # Vehicle has passed
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")

                # Calculate average speed during the capture for logging
                if len(self.vehicle_speeds) > 0:
                    avg_speed = sum(self.vehicle_speeds) / len(self.vehicle_speeds)
                else:
                    avg_speed = DEFAULT_SPEED_KMPH / 3.6

                print(f"🏁 Vehicle {ts} queued. Avg Speed: {avg_speed * 3.6:.1f} km/h")

                # Send data + speed to processor
                processing_queue.put((list(self.current_extracted), ts, avg_speed))

                self.is_stacking = False
                self.speed_estimator.reset()


if __name__ == "__main__":
    system = TollPlazaSystem()
    system.run_forever()
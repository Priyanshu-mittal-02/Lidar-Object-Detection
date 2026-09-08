import serial
import time
import numpy as np
import datetime
import os
import open3d as o3d

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM8'
BAUD_RATE = 460800
POINTS_PER_SCAN = 274
FOV_DEGREES = 96.0
SAVE_DIRECTORY = "./scans"

Z_INCREMENT_METERS = 0.05

# Global storage
all_points_buffer = []
scan_counter = 0

# ==========================================
# OPEN3D VISUALIZER SETUP
# ==========================================
pcd = o3d.geometry.PointCloud()
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="LZR U921 Live View", width=1000, height=700)
vis.add_geometry(pcd)

# ==========================================
# COLORING FUNCTION
# ==========================================
def colorize_points(points):
    """
    Colors points RED if distance is between 0.8 and 1.0 m
    """
    colors = []
    for x, y, z in points:
        d = np.sqrt(x*x + y*y + z*z)
        if 0.8 <= d <= 1.0:
            colors.append([1.0, 0.0, 0.0])   # RED
        else:
            colors.append([0.6, 0.6, 0.6])   # GRAY
    return np.asarray(colors)

# ==========================================
# PCD SAVE FUNCTION
# ==========================================
def save_combined_pcd(points_list, output_folder):
    if not points_list:
        print("[WARNING] No points to save.")
        return

    os.makedirs(output_folder, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_3d_{ts}.pcd")

    with open(filename, 'w') as f:
        f.write("# .PCD v0.7\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points_list)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points_list)}\n")
        f.write("DATA ascii\n")
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[SAVED] {filename}")

# ==========================================
# PROCESS SCAN
# ==========================================
def process_scan(distances_mm):
    global scan_counter

    current_length = scan_counter * Z_INCREMENT_METERS
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    for r, theta in zip(distances_mm, angles):
        r_m = r / 1000.0
        if 0.1 < r_m < 60.0:
            raw_x = r_m * np.cos(theta)
            raw_y = r_m * np.sin(theta)

            x_world = current_length
            y_world = abs(raw_x)
            z_world = raw_y

            all_points_buffer.append((x_world, y_world, z_world))

    scan_counter += 1

    # ===== LIVE VISUAL UPDATE =====
    pts = np.asarray(all_points_buffer)
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colorize_points(pts))

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    if scan_counter % 10 == 0:
        print(f"Scan #{scan_counter} | Points: {len(all_points_buffer)}")

# ==========================================
# MAIN LOOP
# ==========================================
def run_recorder():
    print("LZR U921 LIVE COLORED SCAN (RED = 0.8–1.0m)")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    buffer = b""

    try:
        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

                start = buffer.find(b'\x02')
                if start != -1:
                    buffer = buffer[start:]
                    EXPECTED = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) >= EXPECTED:
                        payload = buffer[4:4 + POINTS_PER_SCAN * 2]
                        distances = [
                            (payload[i] << 8) + payload[i + 1]
                            for i in range(0, len(payload), 2)
                        ]
                        if len(distances) == POINTS_PER_SCAN:
                            process_scan(distances)
                        buffer = buffer[EXPECTED:]

            time.sleep(0.003)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)
        vis.destroy_window()

if __name__ == "__main__":
    run_recorder()

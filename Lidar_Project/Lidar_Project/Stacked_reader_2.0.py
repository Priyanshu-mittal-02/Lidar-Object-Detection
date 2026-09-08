import serial
import time
import numpy as np
import datetime
import os
import signal
import sys

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM7'  # Change to your port
BAUD_RATE = 460800  # Standard LZR U921 baud rate
POINTS_PER_SCAN = 274  # Standard LZR resolution
FOV_DEGREES = 96.0  # Field of View
SAVE_DIRECTORY = "./scans_stacked"  # Folder for combined files

# STACKING SETTINGS
# This simulates movement.
# 0.05 means we assume the sensor moves 5cm between every scan frame.
STACK_INCREMENT = 0.05

# Global buffers
all_points_buffer = []
scan_counter = 0


# ==========================================
# PCD SAVING FUNCTION (COMBINED)
# ==========================================
def save_combined_pcd(points_list, output_folder):
    """
    Saves the ACCUMULATED list of (x, y, z) points into a single PCD file.
    """
    if not points_list:
        print("[WARNING] No points to save.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_folder, f"lzr_stacked_output_{timestamp}.pcd")

    num_points = len(points_list)

    print(f"\n[SAVING] Writing {num_points} points to {filename}...")

    with open(filename, 'w') as f:
        # PCD Header
        f.write("# .PCD v.7 - LZR U921 Stacked Scan\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")

        # PCD Data
        for p in points_list:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"[COMPLETE] File saved successfully.")


# ==========================================
# PROCESSING LOGIC
# ==========================================
def process_frame(distances_mm):
    global scan_counter

    # Calculate the Z-offset for this specific frame (The "Stacking")
    # Frame 0 = 0.0m, Frame 1 = 0.05m, Frame 2 = 0.10m, etc.
    current_z_depth = scan_counter * STACK_INCREMENT

    # Angles for the 274 points
    angles = np.linspace(
        np.radians(-FOV_DEGREES / 2),
        np.radians(FOV_DEGREES / 2),
        len(distances_mm)
    )

    valid_points_in_frame = 0

    for r, theta in zip(distances_mm, angles):
        r_meters = r / 1000.0

        # Filter valid range (0.1m to 60m)
        if 0.1 < r_meters < 60.0:
            # 2D Scan Conversion
            x = r_meters * np.cos(theta)
            y = r_meters * np.sin(theta)

            # 3D Stacking Assignment
            # X = Width, Y = Height, Z = Travel Direction
            z = current_z_depth

            all_points_buffer.append((x, y, z))
            valid_points_in_frame += 1

    scan_counter += 1

    # Simple status update every 10 frames so we know it's working
    if scan_counter % 10 == 0:
        print(f"Stacking Frame #{scan_counter} | Total Points: {len(all_points_buffer)}")


# ==========================================
# MAIN READER LOOP
# ==========================================
def run_recorder():
    ser = None
    print("--- LZR U921 STACKED RECORDER ---")
    print(f"Movement Step: {STACK_INCREMENT}m per scan")
    print("Data is accumulating in RAM. Press Ctrl+C to STOP and SAVE.")

    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}...")

        buffer = b""

        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

                # Find start byte (Assuming 0x02 STX based on your code)
                start_index = buffer.find(b'\x02')

                if start_index != -1:
                    buffer = buffer[start_index:]

                    # Expected size: Header + Data (274*2) + Checksum/Footer
                    EXPECTED_SIZE = (POINTS_PER_SCAN * 2) + 10

                    if len(buffer) >= EXPECTED_SIZE:
                        # Extract payload (Offset 4)
                        raw_payload = buffer[4: 4 + (POINTS_PER_SCAN * 2)]

                        distances = []
                        for i in range(0, len(raw_payload) - 1, 2):
                            high = raw_payload[i]
                            low = raw_payload[i + 1]
                            dist = (high << 8) + low
                            distances.append(dist)

                        # Check if we have a full scan
                        if len(distances) == POINTS_PER_SCAN:
                            process_frame(distances)

                        # Clean buffer
                        buffer = buffer[EXPECTED_SIZE:]

            time.sleep(0.005)

    except serial.SerialException:
        print(f"[ERROR] Could not open {SERIAL_PORT}.")
    except KeyboardInterrupt:
        print("\n[STOP] Recording stopped by user.")
    finally:
        if ser and ser.is_open:
            ser.close()

        # THIS IS WHERE THE MAGIC HAPPENS
        # When you hit Ctrl+C, we dump the whole memory to a file
        save_combined_pcd(all_points_buffer, SAVE_DIRECTORY)


if __name__ == "__main__":
    run_recorder()
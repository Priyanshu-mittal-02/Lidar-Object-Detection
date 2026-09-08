import sys
import os
import struct
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox

# =====================================================
# ALGORITHM CONFIGURATION
# =====================================================
GRID_CELL_SIZE = 0.10


# =====================================================
# FILE I/O HELPERS
# =====================================================
def read_pcd(filename):
    points = []
    if not filename:
        return []

    print(f" Reading: {os.path.basename(filename)}...")
    try:
        with open(filename, 'r') as f:
            data_section = False
            for line in f:
                if line.startswith("DATA ascii"):
                    data_section = True
                    continue
                if data_section:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        try:
                            points.append((float(parts[0]),
                                           float(parts[1]),
                                           float(parts[2])))
                        except ValueError:
                            pass
    except Exception as e:
        messagebox.showerror("Error", str(e))
        return []

    return points


def save_pcd(points, filename):
    if not points:
        messagebox.showwarning("Warning", "No points to save!")
        return

    print(f" Saving: {os.path.basename(filename)}")

    header = (
        "# .PCD v0.7 - SIDE PROFILE OUTPUT\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )

    with open(filename, "w") as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print("✅ Saved successfully")


# =====================================================
# ROTATION (KEY FIX)
# =====================================================
def rotate_points_y_90(points):
    """
    Rotate +90° about Y-axis
    """
    rotated = []
    for x, y, z in points:
        x_new = z
        y_new = y
        z_new = -x
        rotated.append((x_new, y_new, z_new))
    return rotated


# =====================================================
# CORE FILTERING
# =====================================================
def perform_filtering(zero_plane_file, stacked_file, output_file):
    bg_points = read_pcd(zero_plane_file)
    stacked_points = read_pcd(stacked_file)

    if not bg_points or not stacked_points:
        return

    # --- Build background grid ---
    background_matrix = set()
    for x, y, _ in bg_points:
        ix = int(np.floor(x / GRID_CELL_SIZE))
        iy = int(np.floor(y / GRID_CELL_SIZE))
        background_matrix.add((ix, iy))

    # --- Filter objects ---
    object_points = []
    for x, y, z in stacked_points:
        ix = int(np.floor(x / GRID_CELL_SIZE))
        iy = int(np.floor(y / GRID_CELL_SIZE))
        if (ix, iy) not in background_matrix:
            object_points.append((x, y, z))

    print(f"🧩 Object points: {len(object_points)}")

    # 🔥 ROTATE TO SIDE VIEW HERE 🔥
    rotated_points = rotate_points_y_90(object_points)

    save_pcd(rotated_points, output_file)

    messagebox.showinfo(
        "Done",
        f"Side profile saved!\n\nPoints: {len(rotated_points)}"
    )


# =====================================================
# GUI
# =====================================================
def main():
    root = tk.Tk()
    root.withdraw()

    zero_plane = filedialog.askopenfilename(
        title="Select ZERO PLANE",
        filetypes=[("PCD Files", "*.pcd")]
    )
    if not zero_plane:
        return

    stacked = filedialog.askopenfilename(
        title="Select STACKED SCAN",
        filetypes=[("PCD Files", "*.pcd")]
    )
    if not stacked:
        return

    output = filedialog.asksaveasfilename(
        title="Save SIDE VIEW PCD",
        defaultextension=".pcd",
        filetypes=[("PCD Files", "*.pcd")],
        initialfile="side_profile.pcd"
    )
    if not output:
        return

    perform_filtering(zero_plane, stacked, output)


if __name__ == "__main__":
    main()

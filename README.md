# LiDAR-Based Automatic Vehicle Classification (AVC) System

## Overview
This project is a Python-based solution for Automatic Vehicle Classification (AVC) and 3D vehicle profiling using a 2D LiDAR sensor. The system captures raw LiDAR scans over a serial connection, processes the data in real-time, and stacks 2D scans over time (accounting for vehicle speed) to generate comprehensive 3D point cloud (`.pcd`) models of passing vehicles.

## Real-World Application & Impact
While LiDAR is often associated with self-driving cars, this project specifically solves problems in **Intelligent Transportation Systems (ITS)** and **Smart City Infrastructure**. 

To a non-technical user, this project acts as a highly accurate, automated "digital toll-booth operator". Its practical uses include:
- **Automated Toll Billing:** Accurately distinguishing between a small car, an SUV, and a commercial truck so that toll plazas can charge the correct fee without manual human inspection.
- **Traffic Monitoring & Urban Planning:** Helping city planners count and categorize road usage (e.g., how many heavy trucks use a specific bridge daily).
- **Border & Security Checkpoints:** Providing automated 3D dimensional scans of vehicles to detect anomalies (like hidden compartments or oversized cargo) without stopping traffic flow.

By replacing traditional manual auditing or error-prone camera setups with precise laser scanning (LiDAR), this software reduces human error, cuts labor costs, and speeds up traffic flow.

## Key Features
- **Real-Time Data Acquisition:** Reads LiDAR data streams directly via a high-speed serial connection (Baud: 921600).
- **Dynamic Background Calibration:** Learns the background/zero-plane over a series of initial frames (e.g., empty road) to robustly isolate passing vehicles.
- **3D Point Cloud Stacking:** Uses the moving vehicle's speed to convert consecutive 2D slices into accurate 3D representations (`.pcd`).
- **Data Filtering & Outlier Removal:** Implements robust median algorithms (MAD) and Open3D tools to filter out noise, rain, or transient anomalies.
- **Visualization:** Integrates with OpenCV and Open3D to generate Birds-Eye-View (BEV) images and 3D mesh views of the scanned vehicles.

## Tech Stack
- **Python 3.x**
- **Open3D:** For advanced 3D point cloud processing, outlier removal, and rendering.
- **OpenCV (cv2):** For 2D image processing, BEV generation, and coordinate mapping.
- **PySerial:** For hardware interfacing with the LiDAR sensor.
- **NumPy & Matplotlib:** For array manipulations and data plotting.

## Getting Started

### Prerequisites
To run this project, you need Python installed on your machine. Install the required dependencies using the provided `requirements.txt` file:

```bash
pip install -r requirements.txt
```

### Usage
1. Connect your LiDAR sensor to the appropriate COM port (configured as `COM8` by default in the `Master_code.py`).
2. Run the main processing script:
   ```bash
   python Lidar_Project/Lidar_Project/Master_code.py
   ```
3. The system will first run a background calibration phase. Once calibrated, it will actively listen for passing objects/vehicles.
4. Generated 3D point clouds (`.pcd` files) and BEV images will be saved automatically in the designated output directories.

## Project Structure
- `Master_code.py` / `code.py`: The main entry points handling serial reading, processing queues, and threading.
- `Extract_profile.py` / `Zero_plane.py`: Scripts responsible for filtering out the ground/road surface and extracting the actual object shape.
- `stacked_reader.py` / `reader.py`: Utilities to parse, stack, and visualize the generated `.pcd` point clouds.
- `.gitignore`: Configured to exclude massive point cloud datasets, system cache, and image logs from version control.

## Hardware Configuration
- **Start Angle:** -48.0 degrees
- **Angular Resolution:** 0.3516 degrees
- **Operating Range:** 0.10m to 3.5m
- *Note: These settings can be adjusted in the `CONFIGURATION` section of the master script to match your specific LiDAR model.*

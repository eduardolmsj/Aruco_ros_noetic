# ArUco ROS Noetic - USB Cam Detection with Calibration

This repository contains a setup for detecting ArUco markers using the `aruco_ros` package with **ROS Noetic**, a **USB webcam**, and **Docker** for easy deployment and testing.

---

## 📷 Camera Calibration

Before running the ArUco detection node, you must calibrate your camera to obtain the correct intrinsic parameters.

### Steps:

1. Open and run the Jupyter Notebook:

```
notebooks/camera_calibration.ipynb
```

This notebook will guide you through capturing calibration images and generating the camera intrinsic parameters.

2. After calibration, save the resulting YAML file to:

```
camera_calibration/my_webcam.yaml
```

The ArUco detection node will load this file at runtime to correct lens distortion and compute accurate poses.

---

## ▶️ Running the Docker Container and Testing the ArUco Detection Node

All necessary build and run commands are provided in:

```
cmd.txt
```

This file includes:

* Docker image build instructions
* Docker container run commands
* ROS launch command for the ArUco detection node

By following these commands, you can launch the entire system inside Docker and test ArUco detection using your webcam.

---

## 🧱 Project Structure

```
.
├── camera_calibration/
│   └── my_webcam.yaml              # Camera intrinsic parameters (after calibration)
├── camera_calibration.ipynb    # Jupyter Notebook for camera calibration
├── Dockerfile                  # Docker setup for ROS Noetic and aruco_ros
├── launch/
│   └── usb_cam_aruco.launch        # Launch file to start webcam and ArUco detection node
├── cmd.txt                         # Build and run commands for Docker and ROS
└── ...
```

---

## ✅ Requirements

* Docker installed
* USB webcam connected
* Printed or displayed ArUco markers (for detection testing)

---

## ✅ Usage Flow

1. **Calibrate your camera** → Save results to `camera_calibration/my_webcam.yaml`
2. **Build and run the Docker container** → Follow steps in `cmd.txt`
3. **Launch the ArUco detection node** → Using the provided launch file
4. **Point your webcam at an ArUco marker** and observe detection results (it can be seen through **rqt_image_view** in the **aruco_marker_publisher/result** topic)

The results are published to /aruco_marker_publisher/markers, like this:

```
header: 
  seq: 0
  stamp: 
    secs: 1751980672
    nsecs: 969804506
  frame_id: "usb_cam"
id: 115
pose: 
  pose: 
    position: 
      x: -0.00266
      y:  0.01967
      z:  0.44075
    orientation: 
      x: 0.98798
      y: -0.01915
      z: 0.02042
      w: -0.15197
  covariance: [ ... ]
```

---

## ✅ Notes

* Accurate calibration significantly improves detection stability and pose estimation.
* Displaying markers on computer screens may cause detection issues due to flickering, anti-aliasing, or poor viewing angles. Printed markers are recommended for better results.

---

# 🚘 Lane Departure Warning System – BE Project

![Sample Output](https://github.com/DevanshShukla1/Lane_departure_warning/raw/main/new1.png)
![Sample Output](https://github.com/DevanshShukla1/Lane_departure_warning/raw/main/new2.png)



This project implements a **Lane Departure Warning System** designed for Indian road conditions. It uses **CNNs**, **OpenCV**, and **Kalman Filter-based sensor fusion** to detect lane lines, estimate obstacle distances, and generate alerts. The system runs in both the **CARLA simulator** and on a **Raspberry Pi** with live sensors.

#Contributors
Devansh Shukla 
Khushal Thanvi 
Laxmiraman Gudewar

---

## 🔧 Setup & Installation

### 🛠 Environment
- Python 3.7+
- Install dependencies from `requirements.txt`:
  ```bash
  pip install -r requirements.txt
  ```

### 🚗 Simulation Setup (CARLA)
- Install [CARLA 0.9.x](https://carla.org/)
- Start the CARLA server:
  ```bash
  ./CarlaUE4.sh
  ```
- Run the simulation using the provided Python scripts

### 🍓 Hardware Setup (Raspberry Pi)
- Connect:
  - USB or Pi Camera
  - HC-SR04 Ultrasonic Sensor
  - MPU9250 IMU
- Interface sensors via GPIO/I2C
- Run the system using:
  ```bash
  python main.py
  ```

### 🧠 Model Training
- Train a **U-Net** or **SCNN** model on the **LaneNet** dataset
- Save the model as:
  ```bash
  lane_segmentation_model.h5
  ```
  and place it in the project root

### ⚙️ Configuration
- Modify `main.py` to set:
  - Model path
  - PID controller gains
  - Frame rate (FPS)

---

## 🧠 System Architecture

### 🟦 Lane Detection (`lane_detector.py`)
- CNN segments lane regions
- OpenCV Hough Transform fits lane lines
- Computes offset from lane center

### 🟧 Sensor Fusion (`sensor_fusion.py`)
- Kalman filter fuses:
  - Camera-based distance
  - Ultrasonic readings
  - IMU acceleration
- Output: Smoothed, reliable obstacle distance

### 🟩 Control System (`pid_controller.py`)
- PID controller minimizes lateral lane offset
- Generates steering corrections
- Triggers departure alert if offset exceeds threshold

### 🔁 Data Pipeline Overview
```
Camera ➝ CNN ➝ Lane Mask ➝ OpenCV ➝ Offset ➝ PID ➝ Steering
Ultrasonic + IMU ➝ Kalman Filter ➝ Smoothed Distance ➝ Alert
```

---

## 📊 Results & Performance

- ✅ **38% Reduction** in RMSE & MAE with sensor fusion
- ✅ **10% Improvement** in obstacle distance accuracy
- ✅ **95%+ Lane Detection Accuracy** in diverse conditions
- ✅ **< 0.1m Lane Error** with consistent alert generation
- ✅ **Raspberry Pi Compatible** for real-time deployment

---

## 🚀 How to Use

### 🔁 Simulation (CARLA)
```bash
python main.py
```
- Displays live camera feed with overlayed lane markings
- Shows steering angles and fused distance estimation

### 💻 Real Hardware (Raspberry Pi)
- Connect the camera and sensors
- Run `main.py` to start real-time detection, fusion, and alerts

---

## 🖼 Output Samples *(To be added by you)*

- 📸 Lane Detection Screenshot: `figures/sample_lane.png`
- 🗺️ System Flowchart: `figures/flowchart.png`

---

## 📁 References & Links

- 📄 [Final Project Report](https://drive.google.com/file/d/1SS0ge12Tv2Tw1IIAmbQRcIQPPEuCpQ-Z/view?usp=sharing)
- 🛠️ [Debugging Moment – Hardware + Software](https://drive.google.com/file/d/1SLndaCy9OinF8_4DfH-0Wq23ZRouUmL4/view?usp=drive_link)

> _That satisfying moment when all the debugging paid off and everything just worked — hardware and software finally in sync!_

import time
import cv2
try:
    import carla
except:
    carla = None
try:
    import RPi.GPIO as GPIO
except:
    GPIO = None

from lane_detector import LaneDetector
from sensor_fusion import KalmanFusion
from pid_controller import PIDController

# --- Configuration ---
CAMERA_MODEL = 'lane_segmentation_model.h5'
FPS = 10  # target framerate
dt = 1.0 / FPS

def read_ultrasonic():
    """
    Stub for reading ultrasonic sensor (HC-SR04).
    On Raspberry Pi, use GPIO trigger/echo.
    """
    # In simulation or testing, return a fake distance or skip
    return 5.0  # meters (example)

def read_imu():
    """
    Stub for reading IMU acceleration (forward axis).
    """
    # Return current longitudinal acceleration (m/s^2)
    return 0.0

def main():
    # Initialize lane detector, sensor fusion, and PID
    detector = LaneDetector(CAMERA_MODEL)
    kalman = KalmanFusion(dt=dt)
    pid = PIDController(kp=0.05, ki=0.0, kd=0.01)

    # Initialize camera capture (Pi camera or USB cam)
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)  # warm-up camera

    # If using CARLA, set up sensors
    if carla is not None:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()
        # Spawn a vehicle and attach camera/IMU (not fully implemented here)
        # ...
        # (One could attach sensor.actor and call sensor.listen() for callback)
        print("CARLA connected:", world.get_map().name)

    print("Starting main loop...")
    while True:
        start = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        # Lane detection
        offset, mask, lines = detector.process(frame)
        # Sensor fusion
        ultra_dist = read_ultrasonic()
        imu_accel = read_imu()
        # (Optionally compute camera distance from vision)
        fused_dist = kalman.fuse(cam_dist=0.0, ultra_dist=ultra_dist, imu_accel=imu_accel)

        # Control steering using PID on lane offset
        if offset is not None:
            # Positive offset means lane center is to the right; steer right (negative angle)
            steer = -pid.compute(error=offset, dt=dt)
        else:
            steer = 0.0  # no lane, keep steering straight or previous
        # Apply steering to simulator or hardware (not shown here)
        print(f"Offset: {offset}, Fused Dist: {fused_dist:.2f}, Steering cmd: {steer:.3f}")

        # Visualization (optional)
        # overlay lane mask and lines on frame
        color_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(frame, 1.0, color_mask, 0.3, 0)
        cv2.imshow("Lane Overlay", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Enforce loop rate
        elapsed = time.time() - start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

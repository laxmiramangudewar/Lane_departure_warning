import numpy as np

class KalmanFusion:
    """
    Kalman filter for fusing distance measurements from camera, ultrasonic, and IMU.
    State: [distance, velocity]
    """
    def __init__(self, dt):
        # Define Kalman matrices for 2D state [d, v] with distance measurement
        self.dt = dt
        # State transition
        self.F = np.array([[1, dt],
                           [0, 1]])
        # Control-input model (use acceleration from IMU if available)
        self.B = np.array([[0.5*dt**2],
                           [dt]])
        # Measurement matrix (we measure distance only)
        self.H = np.array([[1, 0]])
        # Process and measurement covariance
        self.Q = np.array([[1e-2, 0],
                           [0, 1e-1]])
        self.R = np.array([[1e-1]])  # measurement noise
        # Initial state
        self.x = np.zeros((2,1))
        self.P = np.eye(2)
    
    def predict(self, accel=0.0):
        """
        Predict next state using IMU acceleration.
        """
        # Apply control input (acceleration) as affecting velocity
        u = np.array([[accel]])
        # State prediction: x = F x + B u
        self.x = self.F @ self.x + self.B * u
        # Covariance prediction: P = F P F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """
        Update step with distance measurement z (e.g. from ultrasonic or camera).
        """
        # Measurement residual
        y = np.array([[z]]) - self.H @ self.x
        # Residual covariance
        S = self.H @ self.P @ self.H.T + self.R
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # State update
        self.x = self.x + K @ y
        # Covariance update
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def fuse(self, cam_dist, ultra_dist, imu_accel):
        """
        Fuse sensor readings: assume we take ultrasonic as primary measurement,
        and use camera distance as additional measurement if needed.
        """
        # 1) Predict with IMU
        self.predict(accel=imu_accel)
        # 2) Use ultrasonic measurement
        self.update(ultra_dist)
        # (Optionally update again with camera measurement)
        # self.update(cam_dist)
        # Return fused distance estimate
        return float(self.x[0][0])

import numpy as np

class TrackedObject:
    def __init__(self, initial_state, initial_covariance, process_noise_cov, measurement_noise_cov):
        self.state = initial_state  # Initial state [x, y, z, vx, vy, vz]
        self.covariance = initial_covariance  # Initial covariance matrix
        self.process_noise_cov = process_noise_cov  # Process noise covariance matrix
        self.measurement_noise_cov = measurement_noise_cov  # Measurement noise covariance matrix

    def predict(self):
        # Prediction step
        # State prediction
        F = np.array([[1, 0, 0, 1, 0, 0],
                      [0, 1, 0, 0, 1, 0],
                      [0, 0, 1, 0, 0, 1],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        self.state = np.dot(F, self.state)
        
        # Covariance prediction
        Q = self.process_noise_cov
        self.covariance = np.dot(np.dot(F, self.covariance), F.T) + Q

    def update(self, measurement):
        # Update step
        # Kalman gain calculation
        H = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        R = self.measurement_noise_cov
        S = np.dot(np.dot(H, self.covariance), H.T) + R
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(S))
        
        # State update
        Z = measurement
        self.state = self.state + np.dot(K, Z - np.dot(H, self.state))
        
        # Covariance update
        self.covariance = self.covariance - np.dot(np.dot(K, H), self.covariance)
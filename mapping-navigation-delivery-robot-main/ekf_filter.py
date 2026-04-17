import numpy as np
from constants import ENCODER_ACCEL_NOISE, ENCODER_GYRO_NOISE

def normalize_angle(angle_deg):
    return (angle_deg + np.pi) % (2 * np.pi) - np.pi

# ==========================================================
#                        EKF PREDICT
# ==========================================================
def ekf_predict(x, P, u, dt):
    v, omega = u
    theta = x[2]

    #State prediction
    x_pred = np.array([
        x[0] + v * np.cos(theta) * dt,
        x[1] + v * np.sin(theta) * dt,
        normalize_angle(theta + omega * dt)
    ])

    #Jacobian of motion model
    F = np.array([
        [1, 0, -v * np.sin(theta) * dt],
        [0, 1, v * np.cos(theta) * dt],
        [0, 0, 1]
    ])

    #Process noise covariance

    sigma_x = 0.5 * ENCODER_ACCEL_NOISE * dt ** 2
    sigma_theta = ENCODER_GYRO_NOISE * dt #Already in radians from constants

    Q = np.diag([sigma_x**2, sigma_x**2, sigma_theta**2])
    P_pred = F @ P @ F.T + Q

    return x_pred, P_pred

def observation_model(x_pred):
    return np.array([x_pred[:2]]) #only x and y

# ==========================================================
#                        EKF CORRECT
# ==========================================================
def ekf_correct(x_pred, P_pred, z, R):
    h = observation_model(x_pred) #returns [x, y]
    z = np.array([z]) #make z a 2D array for consistency

    """Jacobian of observation model h(x) = x 
    H = diff(h)/diff(x) = [[1, 0, 0],
                            [0, 1, 0]]
     """
    
    H = np.eye(2,3) 

    y = z - h # Innovation
    S = H @ P_pred @ H.T + R #Innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S) #Kalman Gain

    #State update (3x1 = 3x3 + 3x2 @ 2x1)
    x_new = x_pred + K @ y

    #Keep theta normalized
    x_new[2] = normalize_angle(x_new[2])

    #Covariance update with Joseph form for numerical stability
    I = np.eye(3)
    P_new = (np.eye(3) - K @ H) @ P_pred @ (np.eye(3) - K @ H).T + K @ R @ K.T

    return x_new, P_new


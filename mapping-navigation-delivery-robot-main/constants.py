import numpy as np

#Map Constants
CELL_SIZE = 0.1 #meters per grid cell cell
GRID_SIZE = 200 #20m by 20m map
LOG_ODDS_OCC = np.log(0.9 / (1 - 0.9))
LOG_ODDS_FREE = np.log(0.3 / (1 - 0.3))

#Lidar settinga
LIDAR_MIN_QUALITY = 10 #use best quality of lidar scans
LIDAR_MIN_DIST = 120 #12cm minimum
LIDAR_MAX_DIST = 12000 #12m max distance


#Encoder Constants
GEAR_RATIO = 131.25
Counts_Per_Revolution = 64
CPR = GEAR_RATIO * Counts_Per_Revolution
WHEEL_DIAMETER = 0.210 #wheel diameter in meters
WHEEL_RADIUS = WHEEL_DIAMETER / 2  #wheel radius in meters
WHEEL_BASE = 0.35 #Distance between wheels in meters


#EKF noise (encoder-based)
ENCODER_ACCEL_NOISE = 0.7 #m/s^2
ENCODER_GYRO_NOISE = np.radians(5.0) #deg/s
MEASUREMENT_STD_X = 0.5 * ENCODER_ACCEL_NOISE
MEASUREMENT_STD_Y = MEASUREMENT_STD_X
MEASUREMENT_STD_THETA = ENCODER_GYRO_NOISE

#PWM Limits
MAX_PWM = 60
# Minimum PWM used when enforcing a non-zero threshold (see motors.py)
DEFAULT_MIN_PWM = 30
# When True, small PWM commands within (-DEFAULT_MIN_PWM, DEFAULT_MIN_PWM) are treated
# as zero (deadband). If False, they are clamped to +/- DEFAULT_MIN_PWM.
DEFAULT_DEADBAND = True
# Slew limiter: maximum change in PWM per second (PWM units / second).
# Use this to ramp motor commands smoothly and avoid jerky motion.
PWM_SLEW_RATE = 120.0

#Navigation target speeds for PID tunde for smooth SLAM
TARGET_FORWARD_VEL = 0.08 #m/s
TARGET_ANG_VEL = 0.35 #rad/s (gentle turning so SLAM Map will be clearer) 

#PID gains
KP = 1.2
KI = 0.0 #starting with zero for stability in obstacle avoidance
KD = 0.15

# ============================================
# EXPLORATION PARAMETERS (slam_logic.py)
# ============================================
# Robot physical properties (rectangle footprint)
ROBOT_LENGTH_M = 0.35  # meters, front-to-back (y-axis in local frame)
ROBOT_WIDTH_M = 0.50   # meters, side-to-side (x-axis in local frame)

# Robot polygon footprint (rectangle vertices in local frame, centered at origin)
# Order: [front-left, front-right, back-right, back-left] for counter-clockwise
ROBOT_FOOTPRINT = [
    (ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2),    # front-right
    (ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2),   # back-right
    (-ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2),  # back-left
    (-ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2),   # front-left
]

TARGET_FRONT_MARGIN = 0.10  # meters, extra margin in front when "at target"

# Frontier exploration: circle radius around robot to detect frontier cells
FRONTIER_SEARCH_RADIUS = 0.5  # meters, configurable search radius for frontier-based exploration

# Obstacle detection thresholds
FRONT_BLOCK = 0.45  # meters, below this -> front considered blocked
SIDE_CLEAR = 0.45  # meters, above this -> side considered clear

# Exploration mode settings
EXPLORATION_MODE = "FRONTIER"  # "FRONTIER" or "RHR" (Right-Hand Rule only)
# Encoder-Based EKF SLAM Robot with Frontier Exploration

Real-time Simultaneous Localization and Mapping (SLAM) using Extended Kalman Filter (EKF) with Bayesian occupancy mapping, encoder odometry, and autonomous frontier-based exploration with polygon collision detection and Right-Hand Rule fallback navigation.

## Team Members

- **Primary Developer**: David Oluwasegun Bello
- **Role**: Design of Autonomous robot navigation, Bayesian mapping, exploration logic

---

## Features / Objectives

### Core SLAM
- **EKF State Estimation**: Fuses encoder odometry with lidar scan matching for accurate robot pose (x, y, θ)
- **Bayesian Occupancy Mapping**: Log-odds raycasting with Bresenham line algorithm for real-time occupancy grid generation
- **Scan Matching**: ICP-based registration for lidar-to-lidar loop closure and drift correction
- **Real-time Visualization**: Live trajectory and occupancy map overlays saved every 2 seconds

### Navigation & Exploration
- **Frontier-Based Exploration**: Autonomous exploration toward unexplored regions detected in occupancy grid
- **Polygon Collision Detection**: Rectangle footprint (0.35m × 0.50m) used for safe frontier selection
- **Three-Layer Decision Hierarchy**:
  1. Emergency Maneuver: Reactive RHR when front obstacle < 0.2m
  2. Frontier Navigation: Direct velocity control to polygon-filtered frontier goals (DWA-ready)
  3. RHR Fallback: Wall-following when no frontier found

### Motor Control
- **PID Velocity Tracking**: Proportional-Integral-Derivative control for smooth forward/angular velocity
- **Anti-Windup Integral**: Prevents saturation in velocity error integration
- **Slew Rate Limiting**: Ramped PWM output (max 120 PWM/sec) to prevent jerky motion
- **Deadband/Minimum PWM**: Configurable behavior for small velocity commands

### Data Logging & Export
- **Excel Logging**: Full EKF state (x, y, θ, covariance) + encoder measurements + reference velocities per iteration
- **PNG Visualization**: Trajectory plots, occupancy maps with trajectory overlay
- **Binary Map Export**: PGM format for downstream reinforcement learning or navigation

---

## System Requirements

### Hardware
- **Robot Platform**: Differential drive mobile robot (0.35m length × 0.50m width)
- **Compute**: Raspberry Pi 4 (4GB+ RAM recommended)
- **Lidar**: RPLidar A1/A2 (360° 2D laser scanner, 115200 baud)
- **Encoders**: Two rotary encoders on left/right wheel motors
  - Counts Per Revolution (CPR): 360 (default, adjust in `constants.py`)
  - Wheel radius: 0.05m (default)
  - Wheel base: 0.25m (default)
- **Motors**: PWM-controlled DC motors with direction control (GPIO pins configurable)
- **GPIO Pins** (adjust in `motors.py`):
  - Motor left PWM: GPIO 18, Direction: GPIO 17
  - Motor right PWM: GPIO 23, Direction: GPIO 22
  - Encoder left: GPIO 27 (A), GPIO 17 (B)
  - Encoder right: GPIO 23 (A), GPIO 24 (B)

### Software Requirements
- **Python**: 3.8+
- **OS**: Linux (Raspberry Pi OS, Ubuntu) or macOS (development)
- **Critical Packages**:
  - `numpy`: Numerical computation
  - `opencv-python` (cv2): Image processing for scan matching
  - `rplidar`: Lidar communication
  - `gpiozero`: GPIO control
  - `pandas`: Data logging
  - `matplotlib`: Visualization
  - `imageio`: Image export
  - `pyserial`: Serial communication (for future IMU support)

### Optional
- **IMU Support**: `imu.py` (ESP32-based roll/pitch/yaw) with dedicated `imu_main.py`

---

## Installation & Setup

### 1. Clone/Setup Repository
```bash
cd /Users/abc/Desktop/slam_robot_rebuilt
# Or your project directory
```

### 2. Install Python Dependencies
```bash
pip install numpy opencv-python rplidar gpiozero pandas matplotlib imageio pyserial
```

### 3. Configure Robot Parameters (constants.py)
Edit `/constants.py` to match your robot hardware:
```python
# Motor Pins
MOTOR_LEFT_PWM_PIN = 18      # GPIO pin for left motor PWM
MOTOR_LEFT_DIR_PIN = 17      # GPIO pin for left motor direction
MOTOR_RIGHT_PWM_PIN = 23     # GPIO pin for right motor PWM
MOTOR_RIGHT_DIR_PIN = 22     # GPIO pin for right motor direction

# Encoder Pins & Counts
CPR = 360                    # Counts per revolution per wheel
WHEEL_RADIUS = 0.05          # Meters
WHEEL_BASE = 0.25            # Meters (distance between wheel centers)

# Robot Geometry (for polygon collision detection)
ROBOT_LENGTH_M = 0.35        # Front-to-back (y-axis)
ROBOT_WIDTH_M = 0.50         # Side-to-side (x-axis)

# Exploration Parameters
FRONTIER_SEARCH_RADIUS = 0.5 # Meters
FRONT_BLOCK = 0.3            # Meters (obstacle distance trigger for RHR blending)

# PID Gains
KP = 0.5      # Proportional gain
KI = 0.1      # Integral gain
KD = 0.05     # Derivative gain
MAX_PWM = 60  # Maximum PWM output (0-100)
DEFAULT_MIN_PWM = 10         # Minimum PWM for motor motion
DEFAULT_DEADBAND = True      # Use deadband for small commands

# Velocity Control
TARGET_FORWARD_VEL = 0.3     # m/s
TARGET_ANG_VEL = 1.0         # rad/s
PWM_SLEW_RATE = 120.0        # PWM units per second

# SLAM Parameters
CELL_SIZE = 0.05             # Meters per grid cell
GRID_SIZE = 200              # Grid dimensions (200x200 cells)
EXPLORATION_MODE = "FRONTIER" # "FRONTIER" or "SIMPLE" (RHR only)
```

### 4. Configure Lidar Connection
Ensure lidar is connected via USB:
```bash
# Identify USB port
ls /dev/ttyUSB*

# Update lidar port in main.py if needed (default: /dev/ttyUSB0)
```

### 5. Set GPIO Permissions (Raspberry Pi)
```bash
# Allow gpiozero to access GPIO without sudo
# Add user to dialout/gpio groups
sudo usermod -a -G dialout,gpio $USER
# Log out and back in
```

---

## Usage

### Running Encoder-Based EKF SLAM

#### 1. Basic Execution
```bash
cd /Users/abc/Desktop/slam_robot_rebuilt
python main.py
```

**Expected Output:**
```
[MAIN] Initializing SLAM robot with Bayesian mapping...
[MAIN] Starting lidar...
[Lidar] Connected to /dev/ttyUSB0 at 115200 baud.
[MAIN] Initializing encoders...
[MAIN] Initializing motor controller...
[MAIN] SLAM robot ready. Starting exploration with Bayesian mapping...

[EKF CORRECT] x=0.123m, y=-0.045m, θ=-2.3°
[EXPLORATION] Sectors: L=1.23m, F=0.95m, R=1.10m
[EXPLORATION] Frontier found at (0.50, 0.35) (dist=0.61m)

[VIZ] Saved live_trajectory.png and live_map.png (iteration 50)
[SLAM] Iteration 100
  Pose: x=0.245m, y=0.120m, θ=45.2°
  Odometry: v=0.150m/s, ω=0.312°/s
  Commands: v_ref=0.300m/s, ω_ref=0.500°/s
```

#### 2. Stop Execution
Press `Ctrl+C` to stop robot and save results.

#### 3. Output Files
After shutdown, the following files are generated in the project directory:
- `ekf_slam_log.xlsx`: Complete EKF state history with encoder measurements
- `final_slam_map.png`: Occupancy map with trajectory overlay (10×10 inches @ 100 DPI)
- `map_binary.pgm`: Binary occupancy map for navigation/RL (PBM format)
- `full_trajectory.png`: World-frame trajectory plot
- `live_trajectory.png`: Last live trajectory snapshot
- `live_map.png`: Last live occupancy map snapshot

### Running with Custom Configuration

#### Override Exploration Mode (RHR-only fallback)
```bash
# Edit constants.py before running
EXPLORATION_MODE = "SIMPLE"  # Disables frontier, uses RHR wall-following only

python main.py
```

#### Adjust Motor Gains (for tuning)
```python
# In constants.py, adjust PID parameters
KP = 0.7   # Increase for faster response (may oscillate)
KI = 0.05  # Reduce to prevent integral windup
KD = 0.1   # Increase for damping
PWM_SLEW_RATE = 60.0  # Reduce for smoother acceleration
```

#### Change Lidar Port (if not /dev/ttyUSB0)
```python
# In main.py, line ~235
if not init_lidar(port='/dev/ttyUSB1'):  # Change port here
```

### Testing & Debugging

#### 1. Verify Lidar Connection
```python
# Quick test (run once)
from lidar import init_lidar, get_latest_scan, shutdown_lidar

init_lidar(port='/dev/ttyUSB0')
for i in range(10):
    import time
    time.sleep(0.1)
    scan = get_latest_scan()
    if scan:
        timestamp, points = scan
        print(f"Scan {i}: {len(points)} points")

shutdown_lidar()
```

#### 2. Verify Encoder Readings
```python
# In main.py before running slam_control_loop()
from motors import Encoder, Odometry
import time

enc_left = Encoder(pin_a=27, pin_b=17)
enc_right = Encoder(pin_a=23, pin_b=24)
odom = Odometry(enc_left, enc_right)

# Manually rotate wheels and check output
for i in range(10):
    v, omega, dt = odom.compute()
    print(f"v={v:.3f}m/s, ω={omega:.3f}rad/s, dt={dt:.3f}s")
    time.sleep(0.1)
```

#### 3. Test Motor PWM
```python
from motors import set_motor_speeds, stop_motors
import time

# Set both motors to 50% PWM forward
set_motor_speeds(50, 50)
time.sleep(2)

# Stop
stop_motors()
```

#### 4. View Real-Time Logs
```bash
# While main.py is running, open another terminal
tail -f ekf_slam_log.xlsx  # (won't work directly; use Pandas)

# Or check generated PNGs
display live_trajectory.png  # On Linux/X11
open live_map.png             # On macOS
```

---

## Repository Structure

### Core SLAM Modules
- **`main.py`** (493 lines)
  - Master control loop orchestrating all SLAM components
  - EKF PREDICT: Integrates encoder odometry
  - EKF CORRECT: Fuses lidar scan matching measurements
  - Bayesian occupancy mapping with Bresenham raycasting
  - Frontier detection and exploration decision logic
  - Real-time visualization and Excel logging
  - Entry point: `python main.py`

### Hardware & Odometry
- **`motors.py`** (254 lines)
  - Encoder class: Reads rotary encoder ticks via GPIO
  - Odometry class: Converts encoder ticks to linear (v) and angular (ω) velocity
  - PIDController: Proportional-Integral-Derivative control with anti-windup
  - VelocityController: Converts (v_ref, ω_ref) to differential drive PWM with slew limiting
  - Motor control functions: `set_motor_speeds()`, `stop_motors()`

- **`lidar.py`** (97 lines)
  - LidarReader class: Thread-safe serial communication with RPLidar
  - Angle normalization (degrees → radians, [-π, π))
  - Quality filtering and scan buffering
  - Global convenience functions: `init_lidar()`, `get_latest_scan()`, `shutdown_lidar()`

### SLAM & Estimation
- **`ekf_filter.py`**
  - `ekf_predict(x_est, P_est, u_pred, dt)`: Predict robot pose from odometry
  - `ekf_correct(x_pred, P_pred, z_lidar, R_lidar)`: Update pose with lidar measurement
  - Implements proper 2×3 Jacobian for differential drive kinematic model
  - Joseph form covariance update for numerical stability

- **`scan_matcher.py`**
  - `lidar_to_image(scan)`: Converts polar scan to Cartesian image
  - `estimate_shift_translation(prev_img, curr_img)`: ICP-based scan registration
  - Returns (dx, dy) for EKF measurement update (no dθ to avoid motor jitter)

### Navigation & Exploration
- **`slam_logic.py`** (617 lines)
  - `detect_frontiers()`: Identifies free cells adjacent to unknown regions
  - `get_frontier_goal()`: Selects nearest reachable frontier with polygon collision checking
  - `get_frontier_goal_for_dwa()`: Returns frontier metadata for DWA planner (future integration)
  - `plan_trajectory_to_frontier()`: Velocity controller to frontier (direct or DWA strategy)
  - `exploration_with_fallback()`: Main orchestrator with 3-layer decision hierarchy:
    1. Emergency maneuver (RHR if front < 0.2m)
    2. Frontier exploration (polygon-filtered)
    3. RHR fallback (no frontier)
  - `extract_sector_distances()`: Front/Left/Right/Back lidar sectors
  - `right_hand_rule_decision()`: Wall-following logic
  - Collision detection: `is_polygon_free()` (rectangle), `is_circle_free()` (radius)

### Configuration & Constants
- **`constants.py`**
  - Hardware pins (motors, encoders)
  - Robot geometry (ROBOT_LENGTH_M, ROBOT_WIDTH_M, ROBOT_FOOTPRINT polygon)
  - Motor control parameters (KP, KI, KD, MAX_PWM, PWM_SLEW_RATE)
  - Exploration thresholds (FRONTIER_SEARCH_RADIUS, FRONT_BLOCK)
  - SLAM parameters (CELL_SIZE, GRID_SIZE, LOG_ODDS_OCC/FREE)
  - Velocity targets (TARGET_FORWARD_VEL, TARGET_ANG_VEL)

### Optional IMU Support
- **`imu.py`** (349 lines, optional)
  - IMUReader class: Reads roll/pitch/yaw from ESP32 via serial USB
  - IMUOdometry class: Converts Euler angles to (v, ω) estimates
  - Yaw integration for angular velocity: ω = dθ/dt
  - Pitch-based linear velocity: v ≈ g * sin(pitch)
  - Thread-safe buffer and global functions

- **`imu_main.py`** (497 lines, optional)
  - Identical architecture to `main.py` but uses IMU instead of encoders
  - Drop-in replacement for encoder-based SLAM
  - Integration with `imu.py` instead of `motors.py` for odometry

### Testing & Documentation
- **`motors_test_slew.py`**: Unit tests for slew rate limiting and PWM behavior
- **`INTEGRATION_SUMMARY.md`**: High-level system architecture and module interconnections

---

## Project Architecture Summary

```
┌─────────────────────────────────────────────────────────┐
│                   main.py (Master Loop)                 │
└──────┬──────────────────────┬──────────────────────┬────┘
       │                      │                      │
    EKF Filter          Bayesian Mapping      Exploration Logic
    (ekf_filter.py)     (main.py + slam_logic.py)  (slam_logic.py)
       │                      │                      │
   ┌───┴────────┐      ┌──────┴──────┐         ┌────┴────────┐
   │            │      │             │         │             │
Encoders    Scan Matching   Lidar        Frontier    Obstacle
(motors.py) (scan_matcher.py) (lidar.py)  Detection  Avoidance
   │            │      │             │    (Grid)    (RHR/Polygon)
   └────────────┴──────┴─────────────┴─────┴─────────────────┘
                        │
                   Motor Control
                   (motors.py)
                   PWM → Wheels
```

---

## Key Physics Models

### EKF Prediction (Odometry Integration)
```
Δx = v * cos(θ) * dt
Δy = v * sin(θ) * dt
Δθ = ω * dt
```

### Bayesian Log-Odds Mapping
```
log_odds += 0.6    (occupied cell hit)
log_odds -= 0.2    (free cell on ray)
p = 1 / (1 + exp(-log_odds))  (convert to probability)
```

### PID Velocity Control
```
error = v_ref - v_actual
p_term = KP * error
i_term = KI * ∫ error dt  (with anti-windup clamping)
d_term = KD * d(error)/dt
PWM = p_term + i_term + d_term
```

### Polygon Collision Detection
```
Robot rectangle: [0.25, 0.175] to [-0.25, -0.175] (local frame)
Check: all corners within occupancy grid bounds and non-occupied
```

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Lidar not detected | USB port mismatch | Check `ls /dev/ttyUSB*` and update `main.py` line |
| GPIO permission denied | User not in gpio group | `sudo usermod -a -G gpio $USER` and log out/in |
| Encoders read zero | GPIO pin mismatch | Verify pin numbers in `constants.py` match hardware |
| Robot drifts in EKF | Poor scan matching | Check lidar mounting angle, increase `R_lidar` covariance |
| Motors too jerky | High slew rate / low damping | Reduce `PWM_SLEW_RATE`, increase `KD` |
| Frontier exploration fails | Grid resolution mismatch | Increase `CELL_SIZE` or `FRONTIER_SEARCH_RADIUS` |
| Excel export fails | Pandas not installed | `pip install pandas` |
| Visualization not saving | Matplotlib backend issue | Use headless: `matplotlib.use("Agg")` (already set) |

---

## Future Enhancements

1. **Dynamic Window Approach (DWA)**: Replace direct velocity control with DWA trajectory sampling
   - Framework already in place in `slam_logic.py`
   - DWA planner module: `plan_dwa_trajectory()`

2. **Loop Closure Detection**: Detect revisited areas using scan matching peaks or feature descriptors

3. **Multi-Robot SLAM**: Coordinate mapping and exploration between robots

4. **Semantic Segmentation**: Add object recognition to occupancy grid

5. **Active Frontier Selection**: Prioritize frontiers by exploration gain (uncertainty reduction)

6. **Distance Transform**: Precompute distance field for faster collision checks

---

## References

- **EKF SLAM**: Thrun, Burgard, Fox - "Probabilistic Robotics" (2005)
- **Bayesian Occupancy Mapping**: Elfes, A. (1989) "Occupancy Grids for Mobile Robot Perception"
- **Scan Matching**: ICP algorithm (Besl & McKay, 1992)
- **Frontier Exploration**: Yamauchi, B. M. (1997) "A Frontier-Based Approach for Autonomous Exploration"

---

## License

[Specify your license here - MIT, Apache 2.0, GPL, etc.]

---

## Contact & Support

For questions, issues, or contributions, please contact the development team or open an issue in the repository.

**Last Updated**: December 4, 2025
**Version**: 1.0 (Encoder-based EKF SLAM with Frontier Exploration)

"""
Main SLAM Robot Control Loop with Bayesian Occupancy Mapping

Integrates:
1. Lidar scan reading (lidar.py)
2. Scan matching for position correction (scan_matcher.py)
3. EKF filter for state estimation (ekf_filter.py)
4. Motor control with PID (motors.py)
5. Frontier-based exploration with RHR fallback (slam_logic.py)
6. Bayesian log-odds occupancy mapping with Bresenham raycasting
7. Real-time visualization and data logging

Architecture:
- EKF PREDICT: encoder odometry + motion model
- SCAN MATCHING: lidar to lidar (dx, dy only, no dtheta to avoid motor jitter)
- EKF CORRECT: fuse lidar measurements with odometry
- BAYESIAN MAPPING: log-odds update on each ray (free cells + hit cells)
- SLAM LOGIC: frontier exploration from occupancy grid or obstacle avoidance
- MOTOR CONTROL: velocity tracking with PID + slew limiting
- VISUALIZATION: live trajectory and map overlays (saved PNG every 2s)
- LOGGING: full EKF state + lidar measurements to Excel

Real-time SLAM without pre-saved maps. Frontier-based exploration with
live Bayesian mapping. Custom navigation and DWA compatible.
"""

import time
import numpy as np
import threading
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import imageio
from collections import deque

# Import all modules
from lidar import init_lidar, get_latest_scan, shutdown_lidar
from motors import (
    Encoder, Odometry, VelocityController, stop_motors, set_motor_speeds,
    encoder_thread, encoder_buffer, encoder_lock
)
from scan_matcher import lidar_to_image, estimate_shift_translation
from ekf_filter import ekf_predict, ekf_correct
from slam_logic import (
    process_lidar_for_exploration, process_lidar_simple,
    get_frontier_goal
)
from constants import (
    CPR, WHEEL_RADIUS, WHEEL_BASE,
    CELL_SIZE, GRID_SIZE,
    TARGET_FORWARD_VEL, TARGET_ANG_VEL,
    EXPLORATION_MODE,
    LIDAR_MIN_QUALITY,
)

# ============================================================
# BAYESIAN MAPPING PARAMETERS
# ============================================================
LOG_ODDS_OCC = 0.6      # log-odds increment for occupied cell
LOG_ODDS_FREE = -0.2    # log-odds increment for free cell
log_odds_map = np.zeros((GRID_SIZE, GRID_SIZE))  # Bayesian occupancy grid

# ============================================================
# GLOBAL STATE
# ============================================================

# SLAM state
x_est = np.array([0.0, 0.0, 0.0])  # [x, y, theta] in meters and radians
P_est = np.eye(3) * 0.1  # State covariance

# Lidar buffers
prev_lidar_image = None
grid_lock = threading.Lock()

# Trajectory logging
trajectory_x = []
trajectory_y = []
log_data = []

# Control state
running = True


# ============================================================
# OCCUPANCY GRID UTILITIES
# ============================================================

def world_to_map(xw, yw):
    """Convert world coordinates (meters) to grid indices."""
    mx = int(xw / CELL_SIZE + GRID_SIZE / 2)
    my = int(yw / CELL_SIZE + GRID_SIZE / 2)
    return mx, my


def bresenham(x0, y0, x1, y1):
    """Bresenham line algorithm: returns list of (x, y) grid cells."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cells = []
    
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    
    return cells


def bayesian_occupancy_update(scan, x_robot, y_robot, theta_robot):
    """
    Bayesian log-odds update for occupancy grid.
    
    For each lidar ray:
    1. Mark all cells from robot to hit point as FREE (log-odds -= 0.2)
    2. Mark hit point as OCCUPIED (log-odds += 0.6)
    
    This implements proper Bayesian mapping with Bresenham raycasting.
    
    Args:
        scan: lidar scan (list of (quality, angle_rad, dist_mm) tuples)
        x_robot, y_robot: robot pose in world coords (meters)
        theta_robot: robot heading in radians
    """
    global log_odds_map
    
    mx0, my0 = world_to_map(x_robot, y_robot)
    
    for q, angle_rad, dist_mm in scan:
        # Quality filter
        if q < LIDAR_MIN_QUALITY:
            continue
        
        # Distance filter (100mm to 8m)
        if dist_mm < 100 or dist_mm > 8000:
            continue
        
        # Convert to world frame
        total_angle = theta_robot + angle_rad
        dist_m = dist_mm / 1000.0
        
        hit_x = x_robot + dist_m * np.cos(total_angle)
        hit_y = y_robot + dist_m * np.sin(total_angle)
        
        # Bresenham raycasting
        mx1, my1 = world_to_map(hit_x, hit_y)
        cells = bresenham(mx0, my0, mx1, my1)
        
        # FREE cells (all except last)
        for cx, cy in cells[:-1]:
            if 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE:
                log_odds_map[cy, cx] += LOG_ODDS_FREE
        
        # OCCUPIED cell (last cell, the hit)
        if 0 <= mx1 < GRID_SIZE and 0 <= my1 < GRID_SIZE:
            log_odds_map[my1, mx1] += LOG_ODDS_OCC
    
    # Clip log-odds to prevent overflow
    log_odds_map = np.clip(log_odds_map, -5, 5)


def convert_log_odds_to_occupancy_grid():
    """Convert log-odds map to occupancy probabilities for frontier detection."""
    # p = 1 / (1 + exp(-log_odds))
    occupancy = 1 - 1 / (1 + np.exp(log_odds_map))
    
    # Discretize: >0.6 = occupied, <0.4 = free, else = unknown
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    grid[occupancy > 0.6] = 100   # occupied
    grid[occupancy < 0.4] = 1      # free
    grid[(occupancy >= 0.4) & (occupancy <= 0.6)] = 0  # unknown
    
    return grid


# ============================================================
# VISUALIZATION & LOGGING SETUP
# ============================================================

# Live trajectory plot (world frame)
fig_traj, ax_traj = plt.subplots(figsize=(8, 8))
traj_xy_line, = ax_traj.plot([], [], 'b.-', label="Trajectory")
ax_traj.set_xlim(-5, 5)
ax_traj.set_ylim(-5, 5)
ax_traj.set_title("Live Robot Trajectory (World Frame)")
ax_traj.set_xlabel("X [m]")
ax_traj.set_ylabel("Y [m]")
ax_traj.grid(True)
ax_traj.plot(0.0, 0.0, 'go', markersize=6, label="Start")
ax_traj.legend(loc="upper right")

# Live occupancy grid + trajectory overlay
fig_map, ax_map = plt.subplots(figsize=(8, 8))
p_init = 1 - 1 / (1 + np.exp(log_odds_map))
map_im = ax_map.imshow(p_init, cmap='gray_r', origin='lower', vmin=0.0, vmax=1.0)

start_mx, start_my = world_to_map(0.0, 0.0)
ax_map.plot(start_mx, start_my, 'go', markersize=4, label="Start")

traj_map_line, = ax_map.plot([], [], 'r.-', linewidth=1, label="Trajectory")
ax_map.set_title("Live Occupancy Grid + Robot Trajectory")
ax_map.set_xlabel("Grid X")
ax_map.set_ylabel("Grid Y")
ax_map.grid(True, alpha=0.3)
ax_map.set_xlim(0, GRID_SIZE)
ax_map.set_ylim(0, GRID_SIZE)
ax_map.legend(loc="upper right")

last_map_update = time.time()
LIVE_MAP_INTERVAL = 2.0  # seconds


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

def slam_control_loop():
    """Main SLAM and exploration control loop."""
    global x_est, P_est, prev_lidar_image, log_odds_map, running
    global trajectory_x, trajectory_y, log_data
    
    print("[MAIN] Initializing SLAM robot with Bayesian mapping...")
    
    # Initialize hardware
    print("[MAIN] Starting lidar...")
    if not init_lidar(port='/dev/ttyUSB0'):
        print("[MAIN] ERROR: Failed to initialize lidar. Exiting.")
        return
    
    # Initialize encoder odometry
    print("[MAIN] Initializing encoders...")
    try:
        enc_left = Encoder(pin_a=27, pin_b=17)
        enc_right = Encoder(pin_a=23, pin_b=24)
        odom = Odometry(enc_left, enc_right)
        
        # Start encoder thread
        enc_thread = threading.Thread(target=encoder_thread, args=(odom,), daemon=True)
        enc_thread.start()
    except Exception as e:
        print(f"[MAIN] ERROR: Failed to initialize encoders: {e}")
        shutdown_lidar()
        return
    
    # Initialize motor control
    print("[MAIN] Initializing motor controller...")
    try:
        velocity_controller = VelocityController(min_pwm=30, deadband=True, slew_rate=120.0)
    except Exception as e:
        print(f"[MAIN] ERROR: Failed to initialize motor controller: {e}")
        shutdown_lidar()
        return
    
    print("[MAIN] SLAM robot ready. Starting exploration with Bayesian mapping...")
    
    dt_nominal = 0.1  # 100 Hz nominal, ~10 Hz actual
    t_last = time.time()
    scan_count = 0
    
    try:
        while running:
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now
            
            if dt < dt_nominal * 0.5:
                time.sleep(dt_nominal - dt)
                continue
            
            # ========== GET LATEST SENSOR DATA ==========
            
            scan_result = get_latest_scan()
            if not scan_result:
                print("[MAIN] Waiting for lidar scan...")
                time.sleep(0.05)
                continue
            
            lidar_timestamp, scan = scan_result
            
            # Encoder odometry
            with encoder_lock:
                if encoder_buffer:
                    enc_t, v_meas, omega_meas, dt_enc = encoder_buffer[-1]
                else:
                    v_meas = 0.0
                    omega_meas = 0.0
                    dt_enc = dt_nominal
            
            # ========== EKF PREDICT ==========
            u_pred = (v_meas, omega_meas)
            x_pred, P_pred = ekf_predict(x_est, P_est, u_pred, dt_enc)
            
            # ========== SCAN MATCHING (LIDAR MEASUREMENT) ==========
            
            current_lidar_image = lidar_to_image(scan)
            
            if prev_lidar_image is not None and scan_count % 5 == 0:
                dx_meas, dy_meas = estimate_shift_translation(prev_lidar_image, current_lidar_image)
                z_lidar = np.array([dx_meas, dy_meas])
                R_lidar = np.diag([0.01, 0.01])
                
                # EKF CORRECT
                x_est, P_est = ekf_correct(x_pred, P_pred, z_lidar, R_lidar)
                print(f"[EKF CORRECT] x={x_est[0]:.3f}m, y={x_est[1]:.3f}m, θ={np.degrees(x_est[2]):.1f}°")
            else:
                x_est = x_pred
                P_est = P_pred
            
            prev_lidar_image = current_lidar_image.copy()
            scan_count += 1
            
            # ========== BAYESIAN OCCUPANCY MAPPING ==========
            
            with grid_lock:
                bayesian_occupancy_update(scan, x_est[0], x_est[1], x_est[2])
            
            # ========== FRONTIER DETECTION & EXPLORATION ==========
            
            # Convert log-odds to discrete occupancy grid for frontier detection
            occupancy_grid = convert_log_odds_to_occupancy_grid()
            
            if EXPLORATION_MODE == "FRONTIER":
                v_ref, omega_ref = process_lidar_for_exploration(
                    scan, occupancy_grid, x_est[0], x_est[1], x_est[2]
                )
                
                frontier_goal = get_frontier_goal(occupancy_grid, x_est[0], x_est[1])
                if frontier_goal:
                    print(f"[EXPLORATION] Frontier goal: {frontier_goal}")
            else:
                v_ref, omega_ref = process_lidar_simple(scan)
            
            # ========== MOTOR CONTROL ==========
            
            left_pwm, right_pwm = velocity_controller.compute(
                v_ref, omega_ref,
                v_meas, omega_meas
            )
            
            try:
                set_motor_speeds(left_pwm, right_pwm)
            except Exception as e:
                print(f"[MOTORS] Error: {e}")
            
            # ========== UPDATE TRAJECTORY & LOGGING ==========
            
            trajectory_x.append(x_est[0])
            trajectory_y.append(x_est[1])
            
            row = {
                "timestamp": lidar_timestamp,
                "ekf_x": x_est[0],
                "ekf_y": x_est[1],
                "ekf_theta": np.degrees(x_est[2]),
                "v_meas": v_meas,
                "omega_meas": omega_meas,
                "v_ref": v_ref,
                "omega_ref": omega_ref,
            }
            log_data.append(row)
            
            # ========== LIVE VISUALIZATION (every ~2s) ==========
            
            now = time.time()
            if now - last_map_update > LIVE_MAP_INTERVAL:
                last_map_update = now
                
                # Update trajectory plot
                traj_xy_line.set_data(trajectory_x, trajectory_y)
                fig_traj.tight_layout()
                fig_traj.savefig("live_trajectory.png", dpi=80)
                
                # Update map plot
                p_live = 1 - 1 / (1 + np.exp(log_odds_map))
                map_im.set_data(p_live)
                
                traj_mx = []
                traj_my = []
                for wx, wy in zip(trajectory_x, trajectory_y):
                    mx, my = world_to_map(wx, wy)
                    traj_mx.append(mx)
                    traj_my.append(my)
                
                traj_map_line.set_data(traj_mx, traj_my)
                fig_map.tight_layout()
                fig_map.savefig("live_map.png", dpi=80)
                
                print(f"[VIZ] Saved live_trajectory.png and live_map.png (iteration {scan_count})")
            
            # ========== LOGGING ==========
            
            if scan_count % 10 == 0:
                print(f"\n[SLAM] Iteration {scan_count}")
                print(f"  Pose: x={x_est[0]:.3f}m, y={x_est[1]:.3f}m, θ={np.degrees(x_est[2]):.1f}°")
                print(f"  Odometry: v={v_meas:.3f}m/s, ω={np.degrees(omega_meas):.1f}°/s")
                print(f"  Commands: v_ref={v_ref:.3f}m/s, ω_ref={np.degrees(omega_ref):.1f}°/s\n")
    
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping robot...")
    
    except Exception as e:
        print(f"[MAIN] ERROR in control loop: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # ========== SHUTDOWN & SAVE RESULTS ==========
        print("[MAIN] Shutting down...")
        running = False
        stop_motors()
        shutdown_lidar()
        
        # Save log to Excel
        if log_data:
            print("[MAIN] Saving EKF log to Excel...")
            df = pd.DataFrame(log_data)
            df.to_excel("ekf_slam_log.xlsx", index=False)
        
        # Convert log-odds to occupancy probability
        p_map = 1 - 1 / (1 + np.exp(log_odds_map))
        binary_map = (p_map > 0.6).astype(np.uint8)
        
        # Final map with trajectory overlay
        print("[MAIN] Saving final maps...")
        plt.figure(figsize=(10, 10))
        plt.imshow(p_map, cmap='gray_r', origin='lower')
        plt.plot(start_mx, start_my, 'go', markersize=6, label="Start")
        
        traj_mx = []
        traj_my = []
        for wx, wy in zip(trajectory_x, trajectory_y):
            mx, my = world_to_map(wx, wy)
            traj_mx.append(mx)
            traj_my.append(my)
        
        plt.plot(traj_mx, traj_my, 'r.-', linewidth=1.5, label="Trajectory")
        plt.title(f"Final Bayesian EKF SLAM Map + Trajectory ({scan_count} scans)")
        plt.colorbar(label='Occupancy Probability')
        plt.xlabel("Grid X")
        plt.ylabel("Grid Y")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="upper right")
        plt.tight_layout()
        plt.savefig("final_slam_map.png", dpi=100)
        print("  Saved final_slam_map.png")
        
        # Binary map for navigation
        imageio.imwrite("map_binary.pgm", 255 - binary_map * 255)
        print("  Saved map_binary.pgm")
        
        # Trajectory-only plot (world frame)
        plt.figure(figsize=(10, 10))
        plt.plot(trajectory_x, trajectory_y, 'b.-', linewidth=1.5, markersize=3)
        plt.plot(0.0, 0.0, 'go', markersize=8, label="Start")
        plt.plot(trajectory_x[-1] if trajectory_x else 0, 
                 trajectory_y[-1] if trajectory_y else 0, 
                 'r*', markersize=15, label="End")
        plt.title(f"Full EKF Trajectory (World Frame) - {len(trajectory_x)} poses")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="upper right")
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig("full_trajectory.png", dpi=100)
        print("  Saved full_trajectory.png")
        
        print("[MAIN] Done. Results saved to:")
        print("  - ekf_slam_log.xlsx (full EKF log with encoder data)")
        print("  - final_slam_map.png (occupancy map with trajectory)")
        print("  - map_binary.pgm (binary map for navigation/RL)")
        print("  - full_trajectory.png (world-frame trajectory)")
        print("  - live_trajectory.png, live_map.png (last live snapshots)")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == '__main__':
    slam_control_loop()

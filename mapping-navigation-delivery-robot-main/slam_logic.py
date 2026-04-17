import math
import numpy as np
from constants import (
    LIDAR_MIN_QUALITY,
    TARGET_FORWARD_VEL,
    TARGET_ANG_VEL,
    ROBOT_FOOTPRINT,
    
    FRONTIER_SEARCH_RADIUS,
    FRONT_BLOCK,
    SIDE_CLEAR,
    CELL_SIZE,
    GRID_SIZE,
)

# ============================================================
# FRONTIER DETECTION & EXPLORATION
# ============================================================

def detect_frontiers(occupancy_grid, x_robot, y_robot, search_radius=FRONTIER_SEARCH_RADIUS):
    """
    Detect frontier cells (boundary between free and unknown space) within a search radius.
    
    Frontier = free cell adjacent to at least one unknown cell.
    This represents the exploration boundary.
    
    Args:
        occupancy_grid: numpy array, shape (GRID_SIZE, GRID_SIZE)
                        0 = unknown, 1 = free, 100 = occupied
        x_robot, y_robot: robot's world coordinates (meters)
        search_radius: search radius in meters (default from constants)
    
    Returns:
        frontier_cells: list of (x, y) world coordinates of frontier cells
    """
    # Convert robot world coords to grid coords
    grid_x = int(GRID_SIZE / 2 + x_robot / CELL_SIZE)
    grid_y = int(GRID_SIZE / 2 + y_robot / CELL_SIZE)
    
    # Convert search radius to grid cells
    search_cells = int(search_radius / CELL_SIZE)
    
    frontier_cells = []
    
    # Iterate over cells within search radius
    for dy in range(-search_cells, search_cells + 1):
        for dx in range(-search_cells, search_cells + 1):
            # Check if cell is within grid bounds
            cy, cx = grid_y + dy, grid_x + dx
            if not (0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE):
                continue
            
            # Cell must be free (or partially explored)
            if occupancy_grid[cy, cx] == 100:  # Occupied
                continue
            if occupancy_grid[cy, cx] == 0:  # Unknown
                continue
            
            # Check if adjacent to unknown cell (frontier condition)
            is_frontier = False
            for ny, nx in [(cy-1, cx), (cy+1, cx), (cy, cx-1), (cy, cx+1)]:
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                    if occupancy_grid[ny, nx] == 0:  # Unknown neighbor
                        is_frontier = True
                        break
            
            if is_frontier:
                # Convert grid coords back to world coords
                world_x = (cx - GRID_SIZE / 2) * CELL_SIZE
                world_y = (cy - GRID_SIZE / 2) * CELL_SIZE
                frontier_cells.append((world_x, world_y))
    
    return frontier_cells


def get_frontier_goal(occupancy_grid, x_robot, y_robot, search_radius=FRONTIER_SEARCH_RADIUS):
    """
    Get the nearest reachable frontier cell as an exploration goal.
    
    Uses polygon collision detection to filter unreachable frontiers.
    Only returns frontiers that the robot can physically access without collision.
    
    Args:
        occupancy_grid: numpy array (GRID_SIZE x GRID_SIZE)
        x_robot, y_robot: current robot pose (world coords)
        search_radius: search radius in meters
    
    Returns:
        (goal_x, goal_y) or None if no reachable frontier found
    """
    
    frontiers = detect_frontiers(occupancy_grid, x_robot, y_robot, search_radius)
    
    if not frontiers:
        return None
    
    # Filter frontiers: only keep those where robot can fit (polygon check at neutral heading θ=0)
    reachable_frontiers = []
    for fx, fy in frontiers:
        if is_polygon_free(occupancy_grid, fx, fy, 0, ROBOT_FOOTPRINT):
            reachable_frontiers.append((fx, fy))
    
    if not reachable_frontiers:
        print("[FRONTIER] No reachable frontiers found (all blocked by obstacles)")
        return None
    
    # Return nearest reachable frontier to robot
    frontier_goal = min(reachable_frontiers, key=lambda f: (f[0] - x_robot)**2 + (f[1] - y_robot)**2)
    return frontier_goal


def compute_target_velocity_to_frontier(x_robot, y_robot, theta_robot, frontier_goal):
    """
    Compute desired linear and angular velocity to move toward a frontier goal.
    Simple proportional control toward the goal.
    
    Args:
        x_robot, y_robot, theta_robot: current robot pose
        frontier_goal: (goal_x, goal_y) tuple
    
    Returns:
        (v_ref, omega_ref): reference velocities for motor control
    """
    goal_x, goal_y = frontier_goal
    
    # Vector to goal
    dx = goal_x - x_robot
    dy = goal_y - y_robot
    dist_to_goal = math.sqrt(dx**2 + dy**2)
    
    if dist_to_goal < 0.05:  # Goal reached
        return (0.0, 0.0)
    
    # Desired heading to goal
    desired_theta = math.atan2(dy, dx)
    
    # Angle error (normalized to [-π, π])
    angle_error = desired_theta - theta_robot
    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
    
    # Simple proportional control
    v_ref = TARGET_FORWARD_VEL if dist_to_goal > 0.2 else TARGET_FORWARD_VEL * 0.5
    omega_ref = np.clip(angle_error * 0.5, -TARGET_ANG_VEL, TARGET_ANG_VEL)
    
    return (v_ref, omega_ref)


# ============================================================
# LIDAR SECTOR PROCESSING
# ============================================================

def extract_sector_distances(scan):
    """
    Extract minimum and maximum distances from lidar scan in different sectors.
    
    Returns:
        (front_min, right_min, left_min, back_min, front_max)
        All values in meters.
    """
    front_dists = []
    right_dists = []
    left_dists = []
    back_dists = []

    for (q, angle_rad, dist_mm) in scan:
        if q < LIDAR_MIN_QUALITY:
            continue
        
        d = dist_mm / 1000.0
        
        # Convert radians back to degrees for sector classification
        # (Note: angles are already normalized to [-π, π) by lidar.py)
        angle_deg = np.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        
        # Front sector: 315–360 and 0–45
        if 315 <= angle_deg <= 360 or 0 <= angle_deg <= 45:
            front_dists.append(d)
        # Right sector: 225–315
        elif 225 <= angle_deg <= 315:
            right_dists.append(d)
        # Left sector: 45–135
        elif 45 <= angle_deg <= 135:
            left_dists.append(d)
        # Back sector: 135–225
        elif 135 <= angle_deg <= 225:
            back_dists.append(d)

    def safe_val(lst, func, default):
        return func(lst) if lst else default

    front_min = safe_val(front_dists, min, 9.99)
    front_max = safe_val(front_dists, max, 0.0)
    right_min = safe_val(right_dists, min, 9.99)
    left_min = safe_val(left_dists, min, 9.99)
    back_min = safe_val(back_dists, min, 9.99)

    return front_min, right_min, left_min, back_min, front_max


# ============================================================
# RIGHT-HAND RULE FALLBACK
# ============================================================

def right_hand_rule_decision(left, front, right):
    """
    Simple right-hand rule for obstacle avoidance:
    1. If front clear        -> FORWARD
    2. Else if right clear   -> TURN_RIGHT
    3. Else if left clear    -> TURN_LEFT
    4. Else                  -> TURN_LEFT (keep rotating)
    
    Args:
        left, front, right: minimum distances in each sector (meters)
    
    Returns:
        (v_ref, omega_ref): velocity commands
    """
    # FRONT CLEAR
    if front > FRONT_BLOCK:
        print(f"[RHR] Front clear ({front:.2f}m > {FRONT_BLOCK}m) -> FORWARD")
        return (TARGET_FORWARD_VEL, 0.0)

    # FRONT BLOCKED, RIGHT CLEAR
    if right > SIDE_CLEAR:
        print(f"[RHR] Front blocked, right clear ({right:.2f}m > {SIDE_CLEAR}m) -> TURN_RIGHT")
        return (0.0, -TARGET_ANG_VEL)

    # FRONT BLOCKED, RIGHT BLOCKED, LEFT CLEAR
    if left > SIDE_CLEAR:
        print(f"[RHR] Front/Right blocked, left clear ({left:.2f}m > {SIDE_CLEAR}m) -> TURN_LEFT")
        return (0.0, TARGET_ANG_VEL)

    # EVERYTHING BLOCKED
    print(f"[RHR] All blocked -> TURN_LEFT (escape)")
    return (0.0, TARGET_ANG_VEL)


# ============================================================
# MAIN DECISION LOGIC
# ============================================================

def process_lidar_for_exploration(scan, occupancy_grid, x_robot, y_robot, theta_robot):
    """
    Main decision logic combining frontier exploration + RHR fallback.
    
    ARCHITECTURE: Frontier → DWA → RHR
    ───────────────────────────────────
    
    Strategy:
    1. Try frontier-based exploration: find nearest reachable frontier within search radius.
       - Frontiers are polygon-filtered (robot rectangle 0.35m × 0.50m)
       - Only unreachable frontiers are considered (no collision at neutral heading θ=0)
    2. If frontier found: compute velocity command to move toward it (direct control).
       - Future: can integrate DWA for optimal path planning
    3. If no frontier: fall back to right-hand rule (everywhere explored or blocked).
    
    This function is the primary entry point from main.py.
    
    Args:
        scan: lidar scan (list of (quality, angle_rad, dist_mm) tuples)
        occupancy_grid: numpy array, shape (GRID_SIZE x GRID_SIZE), 0=unknown, 1=free, 100=occupied
        x_robot, y_robot, theta_robot: robot pose in world frame
    
    Returns:
        (v_ref, omega_ref): velocity commands for motor control
    
    Decision Flow:
        1. Emergency: front < 0.2m → RHR immediate stop
        2. Frontier: polygon-filtered frontier found → plan and pursue
        3. Fallback: no frontier → RHR wall-following
    
    ROBOT FOOTPRINT:
        - Rectangle: 0.50m wide (x-axis), 0.35m long (y-axis)
        - Vertices in local frame (from constants.ROBOT_FOOTPRINT)
        - Applied to frontier selection via is_polygon_free() check
    """
    # Delegate to new orchestrator (defaults to 'direct' strategy)
    return exploration_with_fallback(scan, occupancy_grid, x_robot, y_robot, theta_robot, 
                                    planning_strategy='direct')


def process_lidar_simple(scan):
    """
    Fallback: simple right-hand rule only (no exploration or grid).
    Use this for basic obstacle avoidance testing.
    
    Args:
        scan: lidar scan
    
    Returns:
        (v_ref, omega_ref): velocity commands
    """
    front, right, left, back, _ = extract_sector_distances(scan)
    
    print(f"[SIMPLE_RHR] L={left:.2f}m, F={front:.2f}m, R={right:.2f}m")
    
    return right_hand_rule_decision(left, front, right)


# ============================================================
# COLLISION DETECTION: CIRCLE FOOTPRINT (Optional enhancement)
# ============================================================

def is_circle_free(occupancy_grid, x_world, y_world, robot_radius_m, cell_size=CELL_SIZE):
    """
    Check if a circular footprint around (x_world, y_world) is free of obstacles.
    
    Circle collision check: returns True if no occupied cells within robot_radius_m.
    
    Args:
        occupancy_grid: numpy array (GRID_SIZE x GRID_SIZE), 0=unknown, 1=free, 100=occupied
        x_world, y_world: robot center position in world coordinates (meters)
        robot_radius_m: collision radius in meters (typically 0.5 * robot_width)
        cell_size: grid cell size in meters (from constants.CELL_SIZE)
    
    Returns:
        True if circle is free (no obstacle cells found), False if collision
    
    Example usage:
        if is_circle_free(occupancy_grid, x_est[0], x_est[1], 0.18):
            print("Safe to move")
        else:
            print("Collision detected")
    """
    grid_h, grid_w = occupancy_grid.shape
    
    # Convert world coordinates to grid indices
    cx = int(grid_w // 2 + x_world / cell_size)
    cy = int(grid_h // 2 + y_world / cell_size)
    
    # Convert radius to grid cells (with ceiling to be conservative)
    r_cells = int(np.ceil(robot_radius_m / cell_size))
    
    # Bounds check: ensure we stay within grid
    x0 = max(0, cx - r_cells)
    x1 = min(grid_w - 1, cx + r_cells)
    y0 = max(0, cy - r_cells)
    y1 = min(grid_h - 1, cy + r_cells)
    
    # Extract sub-grid
    sub = occupancy_grid[y0:y1+1, x0:x1+1]
    
    # Create circular mask: cells within r_cells of (cx, cy)
    ys, xs = np.ogrid[y0:y1+1, x0:x1+1]
    dist_sq = (xs - cx) ** 2 + (ys - cy) ** 2
    circle_mask = dist_sq <= (r_cells ** 2)
    
    # Check if any occupied cell (value >= 50) is within circle
    # Return True (free) if no collisions; False if collision found
    occupied_threshold = 50
    collision_found = np.any((sub >= occupied_threshold) & circle_mask)
    
    return not collision_found


# ============================================================
# COLLISION DETECTION: POLYGON FOOTPRINT (Recommended)
# ============================================================

def rotate_polygon(polygon, angle_rad):
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    rotated = []
    for px, py in polygon:
        rx = px * cos_a - py * sin_a
        ry = px * sin_a + py * cos_a
        rotated.append((rx, ry))
    return rotated


def is_polygon_free(occupancy_grid, x_world, y_world, theta_world, robot_polygon, cell_size=CELL_SIZE):
    grid_h, grid_w = occupancy_grid.shape
    occupied_threshold = 50
    
    # Step 1: Rotate polygon by current heading
    rotated_poly = rotate_polygon(robot_polygon, theta_world)
    
    # Step 2: Transform polygon vertices to world frame (translate)
    world_poly = [(x_world + dx, y_world + dy) for dx, dy in rotated_poly]
    
    # Step 3: Convert all vertices to grid indices
    grid_poly = []
    for wx, wy in world_poly:
        gx = int(grid_w // 2 + wx / cell_size)
        gy = int(grid_h // 2 + wy / cell_size)
        grid_poly.append((gx, gy))
    
    # Step 4: Get bounding box of polygon
    gxs = [gx for gx, gy in grid_poly]
    gys = [gy for gx, gy in grid_poly]
    x_min, x_max = max(0, min(gxs)), min(grid_w - 1, max(gxs))
    y_min, y_max = max(0, min(gys)), min(grid_h - 1, max(gys))
    
    # Step 5: Check all grid cells in bounding box
    # (For accuracy, use point-in-polygon test; for simplicity, check all cells in bbox)
    for gy in range(y_min, y_max + 1):
        for gx in range(x_min, x_max + 1):
            if occupancy_grid[gy, gx] >= occupied_threshold:
                # More accurate point-in-polygon test
                if point_in_polygon(gx, gy, grid_poly):
                    return False
    
    return True


def point_in_polygon(px, py, polygon):
    """
    Check if point (px, py) is inside polygon using ray casting algorithm.
    
    Args:
        px, py: point coordinates
        polygon: list of (x, y) vertices
    
    Returns:
        True if point is inside polygon, False otherwise
    """
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ============================================================
# EXPLORATION DECISION ARCHITECTURE: FRONTIER → DWA → RHR
# ============================================================

def get_frontier_goal_for_dwa(occupancy_grid, x_robot, y_robot, search_radius=FRONTIER_SEARCH_RADIUS):
    """
    Get frontier goal optimized for DWA path planning.
    
    ARCHITECTURE: Frontier-Based Exploration with DWA Integration
    ────────────────────────────────────────────────────────────
    
    Decision hierarchy:
    1. FRONTIER: Detect and select nearest reachable frontier (polygon-filtered)
    2. DWA: Plan collision-free path from robot to frontier goal
    3. RHR: If stuck or no frontier, fall back to Right-Hand Rule (sector-based)
    
    This function handles STEP 1 and returns a goal suitable for DWA.
    
    ROBOT FOOTPRINT (Rectangle):
        Width (side-to-side):  0.50m (x-axis in local frame)
        Length (front-to-back): 0.35m (y-axis in local frame)
        
    Collision filtering:
        - Frontiers checked with is_polygon_free(...) at θ=0 (neutral heading)
        - Only frontiers where entire robot rectangle fits are returned
        - Conservative: no risk of planning unreachable goals
    
    Args:
        occupancy_grid: numpy array (GRID_SIZE x GRID_SIZE), 0=unknown, 1=free, 100=occupied
        x_robot, y_robot: robot center (world coords, meters)
        search_radius: frontier search radius (default 0.5m)
    
    Returns:
        dict with frontier goal info, or None:
        {
            'goal': (x, y),              # frontier coordinates in world frame
            'dist_to_robot': float,      # Euclidean distance to robot
            'is_reachable': bool,        # polygon collision check result
        }
    
    Usage in main.py:
        frontier_data = get_frontier_goal_for_dwa(occupancy_grid, x, y)
        if frontier_data:
            frontier_goal = frontier_data['goal']
            # Pass to DWA planner
            dwa_trajectory = plan_dwa_to_goal(x, y, theta, frontier_goal, occupancy_grid)
    """
    frontier_goal = get_frontier_goal(occupancy_grid, x_robot, y_robot, search_radius)
    
    if frontier_goal is None:
        return None
    
    dist_to_robot = np.sqrt((frontier_goal[0] - x_robot)**2 + (frontier_goal[1] - y_robot)**2)
    
    return {
        'goal': frontier_goal,
        'dist_to_robot': dist_to_robot,
        'is_reachable': True,  # Already filtered by get_frontier_goal via polygon check
    }


def plan_trajectory_to_frontier(occupancy_grid, x_robot, y_robot, theta_robot, frontier_goal, 
                                strategy='direct'):
    """
    Plan trajectory to frontier goal with selected strategy.
    
    STRATEGIES:
    ──────────
    1. 'direct': Simple proportional heading control (fast, for narrow spaces)
       - Compute v_ref, omega_ref pointing directly at frontier
       - Useful when DWA is overkill or during tight navigation
       
    2. 'dwa': Dynamic Window Approach (future integration)
       - Sample velocity pairs (v, ω) within dynamic window
       - Validate each trajectory sample with polygon collision check
       - Select best trajectory balancing progress + obstacle distance
       - Requires: dwa_planner module (not yet implemented)
    
    Args:
        occupancy_grid: numpy array (GRID_SIZE x GRID_SIZE)
        x_robot, y_robot, theta_robot: current robot pose
        frontier_goal: (goal_x, goal_y) tuple
        strategy: 'direct' or 'dwa' (default 'direct')
    
    Returns:
        (v_ref, omega_ref): velocity commands
    """
    if strategy == 'direct':
        return compute_target_velocity_to_frontier(x_robot, y_robot, theta_robot, frontier_goal)
    
    elif strategy == 'dwa':
        # Future: DWA-based planning
        print("[PLANNER] DWA not yet implemented; using direct control")
        return compute_target_velocity_to_frontier(x_robot, y_robot, theta_robot, frontier_goal)
    
    else:
        raise ValueError(f"Unknown strategy: {strategy}. Use 'direct' or 'dwa'")


def exploration_with_fallback(scan, occupancy_grid, x_robot, y_robot, theta_robot, 
                              planning_strategy='direct'):
    """
    Main exploration orchestrator: Frontier → DWA/Direct → RHR
    
    DECISION FLOW:
    ──────────────
    
    1. EMERGENCY CHECK (sector-based, <1ms):
       If front obstacle < 0.2m → immediate RHR fallback
    
    2. FRONTIER SELECTION (grid-based, ~1-5ms):
       Try get_frontier_goal_for_dwa(...)
       - If found: proceed to planning
       - If not found: jump to RHR (everything explored or blocked)
    
    3. TRAJECTORY PLANNING (~5-10ms):
       Use selected strategy:
       - 'direct': proportional control to frontier (always works)
       - 'dwa': Dynamic Window Approach (future, more optimal)
    
    4. OBSTACLE BLENDING (reactive, <1ms):
       If front blocked during planning → increase turn rate
    
    5. FALLBACK: RHR (sector-based, <1ms):
       If no frontier or planning fails → wall-following
    
    ROBOT FOOTPRINT (Rectangle):
        - Width: 0.50m, Length: 0.35m
        - Used in polygon_free checks for frontier filtering
    
    Args:
        scan: lidar scan [(quality, angle_rad, dist_mm), ...]
        occupancy_grid: numpy array (GRID_SIZE x GRID_SIZE)
        x_robot, y_robot, theta_robot: robot pose
        planning_strategy: 'direct' (default) or 'dwa' (future)
    
    Returns:
        (v_ref, omega_ref): velocity commands for motor control
    """
    
    # Extract sector distances for emergency checks and RHR
    front, right, left, back, _ = extract_sector_distances(scan)
    
    print(f"[EXPLORATION] Sectors: L={left:.2f}m, F={front:.2f}m, R={right:.2f}m")
    
    # ─────────────────────────────────────────────────────────────────────
    # PRIORITY 1: EMERGENCY MANEUVER (if front collision imminent)
    # ─────────────────────────────────────────────────────────────────────
    if front < 0.2:
        print(f"[EXPLORATION] EMERGENCY: Front too close ({front:.2f}m) → RHR maneuver to find free space")
        return right_hand_rule_decision(left, front, right)
    
    # ─────────────────────────────────────────────────────────────────────
    # PRIORITY 2: FRONTIER-BASED EXPLORATION
    # ─────────────────────────────────────────────────────────────────────
    frontier_data = get_frontier_goal_for_dwa(occupancy_grid, x_robot, y_robot)
    
    if frontier_data:
        frontier_goal = frontier_data['goal']
        print(f"[EXPLORATION] Frontier found at {frontier_goal} (dist={frontier_data['dist_to_robot']:.2f}m)")
        
        # Plan trajectory to frontier
        v_ref, omega_ref = plan_trajectory_to_frontier(
            occupancy_grid, x_robot, y_robot, theta_robot, frontier_goal, 
            strategy=planning_strategy
        )
        
        # Blend with reactive obstacle avoidance
        if front < FRONT_BLOCK:
            print(f"[EXPLORATION] Front blocked ({front:.2f}m) during frontier pursuit → boost turn rate")
            omega_ref = np.clip(omega_ref + 0.2 * TARGET_ANG_VEL, -TARGET_ANG_VEL, TARGET_ANG_VEL)
        
        return (v_ref, omega_ref)
    
    # ─────────────────────────────────────────────────────────────────────
    # PRIORITY 3: FALLBACK TO RIGHT-HAND RULE
    # ─────────────────────────────────────────────────────────────────────
    print(f"[EXPLORATION] No reachable frontier in search radius → RHR fallback")
    return right_hand_rule_decision(left, front, right)


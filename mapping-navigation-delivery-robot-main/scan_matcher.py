import numpy as np
import cv2
import os
# ==========================================================
#                 LIDAR → OCCUPANCY IMAGE
# ==========================================================
def lidar_to_image(scan, grid_size=200, resolution=0.05): #output image is 200x200 pixels, 5 cm per pixel
    """
    Convert RPLidar A2 scan to 2D image for phase-correlation scan matching.
    Higher-quality points appear in 'scan' as (quality, angle, distance).
    """
    img = np.zeros((grid_size,grid_size), dtype=np.uint8) #200x200 array of zeros(black image), 8-bit unsigned int (pixel from 0-255)
    center = grid_size // 2 #(100, 100) is the center of the image

    for (quality, angle, distance) in scan:
        if distance< 100 or distance>12000:
            continue

        r = distance / 1000.0 #convert mm to meters
        theta = np.radians(angle)

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        ix = int(center + x / resolution) #convert meters (world coords) to pixels (image coords)
        iy = int(center + y / resolution)

        if 0 <= ix < grid_size and 0 <= iy < grid_size:
            img[iy, ix] = max(img[iy, ix], 1) # Simple occupancy: 1 = occupied, 0 = free

    return img


# ==========================================================
#                 SCAN MATCHING: SHIFT TRANSLATION
# ==========================================================
def estimate_shift_translation(img1, img2, resolution=0.05):
    img1f = img1.astype(np.float32)
    img2f = img2.astype(np.float32)

    # XY shift via phase correlation
    shift_xy, _ = cv2.phaseCorrelate(img2f, img1f)
    dx_pix, dy_pix = shift_xy
    dx = dx_pix * resolution
    dy = dy_pix * resolution

    return dx, dy


# ==========================================================
#                 VISUALIZATION: SAVE IMAGES FOR DEBUG
# ==========================================================
def visualise_scans(scan1, scan2, output_dir="./scans", grid_size=200, resolution=0.05):
    """
    Convert two consecutive scans to occupancy images and save as PNG files.
    Useful for inspecting the quality of lidar scans before scan matching.
    
    Args:
        scan1: Previous lidar scan (list of (quality, angle, distance) tuples)
        scan2: Current lidar scan (list of (quality, angle, distance) tuples)
        output_dir: Directory to save PNG files (default "./scans")
        grid_size: Grid size in pixels (default 200×200)
        resolution: Meters per pixel (default 0.05 m/pixel)
    
    Returns:
        None (saves scan1.png and scan2.png to output_dir)
    """
   
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    img1 = lidar_to_image(scan1, grid_size, resolution)
    img2 = lidar_to_image(scan2, grid_size, resolution)
    
    # Scale images to 255 for better visibility in PNG
    img1_display = (img1 * 255).astype(np.uint8)
    img2_display = (img2 * 255).astype(np.uint8)
    
    # Save as PNG files
    path1 = os.path.join(output_dir, "scan1.png")
    path2 = os.path.join(output_dir, "scan2.png")

    cv2.imwrite(path1, img1_display)
    cv2.imwrite(path2, img2_display)
    
    print(f"Saved scan1.png to {path1} ")
    print(f"Saved scan2.png to {path2} ")



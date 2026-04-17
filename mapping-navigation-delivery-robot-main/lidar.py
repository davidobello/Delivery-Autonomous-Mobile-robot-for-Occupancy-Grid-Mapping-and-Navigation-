import threading
import time
import numpy as np
from collections import deque
from rplidar import RPLidar
from constants import LIDAR_MIN_QUALITY

class LidarReader:
    """Thread-safe Lidar reader with error handling and reconnection. """

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, buffer_size=2):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.buffer = deque(maxlen=buffer_size)
        self.lock = threading.Lock()
        self.thread = None
        self.running = False

    def connect(self):
        """Connect to lidar; raise exception if fails."""
        try:
            self.lidar = RPLidar(self.port, baudrate=self.baudrate)
            self.lidar.connect()
            print(f"[Lidar] Connected to {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            print(f"[Lidar] Connection error: {e}")
            return False
        
    def start(self):
        """Start lidar reading thread."""
        if not self.connect():
            return False
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True
    
    def _read_loop(self):
        """Main lidar reading loop."""
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                # Filter scan by quality, normalize angles to [-π, π), and collect valid points
                filtered_scan = []
                for q, angle_deg, dist in scan:
                    if q >= LIDAR_MIN_QUALITY:
                        # Convert degrees to radians
                        angle_rad = np.radians(angle_deg)
                        # Normalize to [-π, π)
                        angle_rad = (angle_rad + np.pi) % (2 * np.pi) - np.pi
                        filtered_scan.append((q, angle_rad, dist))
                
                if filtered_scan:  # Only append non-empty scans
                    with self.lock:
                        self.buffer.append((time.time(), filtered_scan))
        except Exception as e:
            print(f"[LIDAR THREAD] Error: {e}")
        finally:
           self.stop()     
    
    def get_latest_scan(self):
        """Get latest scan or None if buffer is empty."""
        with self.lock:
            if self.buffer:
                return self.buffer[-1]
        return None
    
    def stop(self):
        """Shutdown lidar and clean up resources."""
        self.running = False
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
                print("[LIDAR] Disconnected")
            except Exception as e:
                print(f"[LIDAR] Disconnection error: {e}")


# Global instance for convenience
lidar_reader = None

def init_lidar(port='/dev/ttyUSB0'):
    """Initialize global lidar reader."""
    global lidar_reader
    lidar_reader = LidarReader(port=port)
    return lidar_reader.start()

def get_latest_scan():
    """Get latest scan from global lidar reader."""
    global lidar_reader
    if lidar_reader:
        return lidar_reader.get_latest_scan()
    return None

def shutdown_lidar():
    """Shut down global lidar reader."""
    global lidar_reader
    if lidar_reader:
        lidar_reader.stop()
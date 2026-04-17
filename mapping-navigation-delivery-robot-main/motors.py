# motors.py
import time
import math
import threading
from collections import deque
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder
from constants import CPR, WHEEL_RADIUS, WHEEL_BASE, KP, KI, KD, MAX_PWM, DEFAULT_MIN_PWM, DEFAULT_DEADBAND, PWM_SLEW_RATE

# Motor pins (update with your actual pins)
MOTOR_LEFT_PWM_PIN = 18
MOTOR_LEFT_DIR_PIN = 17
MOTOR_RIGHT_PWM_PIN = 23
MOTOR_RIGHT_DIR_PIN = 22

# Motor setup using gpiozero
motor_left_pwm = PWMOutputDevice(MOTOR_LEFT_PWM_PIN, frequency=1000)
motor_left_dir = DigitalOutputDevice(MOTOR_LEFT_DIR_PIN)
motor_right_pwm = PWMOutputDevice(MOTOR_RIGHT_PWM_PIN, frequency=1000)
motor_right_dir = DigitalOutputDevice(MOTOR_RIGHT_DIR_PIN)

def stop_motors():
    """stop both motors."""
    motor_left_pwm.value = 0
    motor_right_pwm.value = 0

def set_motor_speeds(left_pwm, right_pwm):
    """Set motor speeds. Positive = forward, negative = backward. PWM 0-100."""
    # Left motor
    if left_pwm >= 0:
        motor_left_dir.on()
    else:
        motor_left_dir.off()
    motor_left_pwm.value = min(abs(left_pwm), 100) / 100.0  # gpiozero uses 0.0 - 1.0
    
    # Right motor
    if right_pwm >= 0:
        motor_right_dir.on()
    else:
        motor_right_dir.off()
    motor_right_pwm.value = min(abs(right_pwm), 100) / 100.0


encoder_buffer = deque(maxlen=100)
encoder_lock = threading.Lock()
running = True

def encoder_thread(odometry):
    global running
    while running:
        v, omega, dt = odometry.compute()
        with encoder_lock:
            encoder_buffer.append((time.time(), v, omega, dt))
        time.sleep(0.01)  # 100Hz


# ============================================
# ENCODER class (using gpiozero RotaryEncoder)
# ============================================
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.encoder = RotaryEncoder(pin_a, pin_b, max_steps= None)  # max_steps=0 for unlimited
    
    def read(self):
        return self.encoder.steps


# ============================================
# ODOMETRY class
# ============================================
class Odometry:
    def __init__(self, encoder_left, encoder_right):
        self.enc_l = encoder_left
        self.enc_r = encoder_right

        self.prev_l = 0
        self.prev_r = 0
        self.prev_t = time.time()

    def compute(self):
        now = time.time()
        dt = now - self.prev_t
        self.prev_t = now

        # Read encoder ticks
        new_l = self.enc_l.read()
        new_r = self.enc_r.read()

        delta_l = new_l - self.prev_l
        delta_r = new_r - self.prev_r
        self.prev_l = new_l
        self.prev_r = new_r

        # Convert ticks → linear displacement
        dist_l = 2 * math.pi * WHEEL_RADIUS * (delta_l / CPR)
        dist_r = 2 * math.pi * WHEEL_RADIUS * (delta_r / CPR)

        # Compute robot linear + angular velocity
        v = (dist_l + dist_r) / 2
        omega = (dist_r - dist_l) / WHEEL_BASE

        return v, omega, dt


# ============================================
# PID CONTROLLER class
# ============================================
class PIDController:
    """
    PID controller for smooth velocity/angular velocity control.
    Critical for EKF SLAM - slow, controlled rotation helps scan matching.
    """
    def __init__(self, kp= KP, ki=KI, kd=KD, max_output=MAX_PWM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now 

        if dt <= 0:
            return 0.0 # Prevent division by zero
        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-self.max_output, min(self.max_output, self.integral)) #min makes the integral not exceed max output while max makes it not go below negative max output
        i_term = self.ki * self.integral
    
        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        self.prev_error = error

        # Total output (clamped)
        output = p_term + i_term + d_term
        return max(-self.max_output, min(self.max_output, output))


# ============================================
# VELOCITY CONTROLLER
# ============================================
# Scaling: convert velocity (m/s) to PWM (0-60)
# At MAX_PWM=60, robot moves at roughly 0.3 m/s (adjust based on your robot)
V_TO_PWM_SCALE = MAX_PWM / 0.3  # PWM per m/s
OMEGA_TO_PWM_SCALE = MAX_PWM / 1.0  # PWM per rad/s


class VelocityController:
    """
    Converts (v_ref, omega_ref) to motor PWM.
    Uses feedforward + PID for smooth SLAM-friendly motion.
    """
    def __init__(self, min_pwm=DEFAULT_MIN_PWM, deadband=DEFAULT_DEADBAND, slew_rate=PWM_SLEW_RATE):
        # PID controllers
        self.pid_v = PIDController(kp=KP, ki=KI, kd=KD)
        self.pid_omega = PIDController(kp=KP * 0.5, ki=KI, kd=KD * 0.5) # reduced gains for angular velocity

        # Configurable minimum PWM and deadband behavior to prevent jerking
        # - min_pwm: threshold (absolute PWM) below which outputs are considered "small"
        # - deadband: if True, small outputs within (-min_pwm, min_pwm) are set to 0.0
        #            if False, small non-zero outputs are clamped to +/- min_pwm
        self.min_pwm = min_pwm
        self.deadband = deadband

        # Slew limiter (PWM units / second). When >0, output PWM will be ramped
        # at most `slew_rate` per second to avoid sudden jumps.
        self.slew_rate = float(slew_rate)

        # State for slew limiter
        self.last_left_pwm = 0.0
        self.last_right_pwm = 0.0
        self.last_time = time.time()
    
    def reset(self):
        self.pid_v.reset()
        self.pid_omega.reset()

    def compute(self, v_ref, omega_ref, v_actual, omega_actual):
        """
        Compute motor PWM from reference and actual velocities.
        Returns (left_pwm, right_pwm)
        """
        #feedforward: directly convert reference velocity to base PWM
        base_v_pwm = v_actual * V_TO_PWM_SCALE
        base_omega_pwm = omega_actual * OMEGA_TO_PWM_SCALE

        # PID correction on velocity error
        v_error = v_ref - v_actual
        omega_error = omega_ref - omega_actual
        v_correction = self.pid_v.compute(v_error) # Already in PWM unit
        omega_correction = self.pid_omega.compute(omega_error) # Already in PWM unit
        
        # Total commanded PWM
        v_pwm = base_v_pwm + v_correction
        omega_pwm = base_omega_pwm + omega_correction

        
        # Convert to differential drive (left, right PWM)
        left_pwm = v_pwm - (omega_pwm * WHEEL_BASE / 2)
        right_pwm = v_pwm + (omega_pwm * WHEEL_BASE / 2)
    
        # Clamp to MAX_PWM
        left_pwm = max(-MAX_PWM, min(MAX_PWM, left_pwm))
        right_pwm = max(-MAX_PWM, min(MAX_PWM, right_pwm))

        # Apply configurable deadband / minimum-PWM behavior to prevent jerky small motions
        if self.deadband:
            # Treat small commands as zero (avoids brief jerks when controller oscillates near zero)
            if abs(left_pwm) > 0 and abs(left_pwm) < self.min_pwm:
                left_pwm = 0.0
            if abs(right_pwm) > 0 and abs(right_pwm) < self.min_pwm:
                right_pwm = 0.0
        else:
            # Enforce a minimum non-zero magnitude so motors get enough torque to move
            if 0 < abs(left_pwm) < self.min_pwm:
                left_pwm = math.copysign(self.min_pwm, left_pwm)
            if 0 < abs(right_pwm) < self.min_pwm:
                right_pwm = math.copysign(self.min_pwm, right_pwm)

        # Slew limiter: ramp outputs from previous values toward the new commands
        if self.slew_rate and self.slew_rate > 0.0:
            now = time.time()
            dt_slew = now - self.last_time
            if dt_slew <= 0.0:
                dt_slew = 1e-6
            max_delta = self.slew_rate * dt_slew

            left_delta = left_pwm - self.last_left_pwm
            left_delta = max(-max_delta, min(max_delta, left_delta))
            left_pwm = self.last_left_pwm + left_delta

            right_delta = right_pwm - self.last_right_pwm
            right_delta = max(-max_delta, min(max_delta, right_delta))
            right_pwm = self.last_right_pwm + right_delta

            # Update slew state
            self.last_left_pwm = left_pwm
            self.last_right_pwm = right_pwm
            self.last_time = now
        
        return left_pwm, right_pwm
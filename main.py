import can
import time
from mks_servo_can import MksServo

# Constants
TIMEOUT = 30  # Timeout for waiting for motor idle (seconds)
SPEED = 500  # Speed in units specific to the library (adjust as needed)
ACCEL = 1   # Acceleration in units specific to the library (adjust as needed)
SUBDIVISIONS = 16  # Microstepping resolution

# Initialize the CAN interface
bus = can.interface.Bus(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
notifier = can.Notifier(bus, [])

# Connect to the servos with CAN IDs 1, 2, and 3
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)

def angle_to_ticks(angle_deg):
    return angle_deg * 5.56  # Adjust based on your motor

def ticks_to_angle(ticks):
    return ticks / 5.56

def wait_for_motor_idle(servo, timeout):
    """Wait for a single motor to become idle."""
    print(f"Waiting for motor {servo.can_id} to idle: ", end="", flush=True)
    start_time = time.perf_counter()
    while (time.perf_counter() - start_time < timeout) and servo.is_motor_running():
        print(f"{servo.read_motor_speed()} ", end="", flush=True)
        time.sleep(0.1)
    if servo.is_motor_running():
        print("Timeout")
        return False
    print("Idle")
    return True
'''
def wait_for_all_motors_idle(servos, timeout):
    """Wait for all motors to become idle."""
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < timeout:
        all_idle = True
        for servo in servos:
            if servo.is_motor_running():
                all_idle = False
                break
        if all_idle:
            print("\nAll motors idle")
            return True
        # Print the position of each motor
        speed_str = ', '.join([f"Motor {servo.can_id}: {servo.read_motor_speed()}" for servo in servos])
        print(speed_str, end='\r', flush=True)
        time.sleep(0.1)
    print("\nTimeout waiting for all motors")
    return False
'''
def wait_for_all_motors_idle(servos, timeout):
    start_time = time.perf_counter()
    last_print_time = start_time
    print_interval = 0.5  # Print every 0.5 seconds
    
    while time.perf_counter() - start_time < timeout:
        all_idle = True
        for servo in servos:
            if servo.is_motor_running():
                all_idle = False
                break
        if all_idle:
            print("All motors idle")
            return True
        
        current_time = time.perf_counter()
        if current_time - last_print_time >= print_interval:
            print(f"\nTime: {current_time - start_time:.1f}s")
            for servo in servos:
                speed = servo.read_motor_speed()
                position = servo.read_encoder_value_addition()
                status = "Moving" if servo.is_motor_running() else "Idle"
                print(f"Motor {servo.can_id}: Speed={speed}, Position={position}, Status={status}")
            last_print_time = current_time
        time.sleep(0.1)
    
    print("Timeout waiting for all motors")
    return False

def emergency_stop(servo):
    """Stop a motor immediately."""
    try:
        print(f"Emergency stop motor {servo.can_id}")
        servo.emergency_stop_motor()
    except Exception as e:
        print(f"Error during emergency stop: {e}")

def configure_servo(servo):
    """Configure a servo motor."""
    print(f"Configuring servo {servo.can_id}")
    servo.set_work_mode(MksServo.WorkMode.SrvFoc)  # Field-Oriented Control mode
    servo.set_subdivisions(SUBDIVISIONS)           # Set microstepping
    servo.set_working_current(3000)                # Current in mA (assumed)
    servo.set_current_axis_to_zero()               # Set current position as zero
    #print(servo.read_encoder_value_addition())

def move_motors_to_positions(servos, positions):
    """Move all motors to their target positions simultaneously."""
    for servo, pos in zip(servos, positions):
        print(f"Moving motor {servo.can_id} to position {pos}", flush=True)
        servo.run_motor_absolute_motion_by_axis(SPEED, ACCEL, pos)

def log_errors(servos, target_positions):
    for servo, target in zip(servos, target_positions):
        actual = servo.read_encoder_value_addition()  
        error = target - actual
        print(f"Servo {servo.can_id}: Target={target}, Actual={actual}, Error={error}")
        # Log to file
        with open("errors.txt", "a") as f:
            f.write(f"{servo.can_id}, {target}, {actual}, {error}\n")

def main():
    # List of all servos for convenience
    servos = [servo1, servo2, servo3]

    # Emergency stop all servos
    for servo in servos:
        emergency_stop(servo)
    
    # Calibrate encoders (uncommented and added waiting)
    print("Calibrating encoders...")
    for servo in servos:
        servo.b_calibrate_encoder()
        if not wait_for_motor_idle(servo, TIMEOUT):
            print(f"Calibration failed for motor {servo.can_id}")
            emergency_stop(servo)
            return
    print("Calibration completed.")
    
    # Configure all servos
    for servo in servos:
        configure_servo(servo)

    # Define target positions for two poses (example values)
    pose1 = [100000, 200000, 300000]  # Extended position
    pose2 = [0, 0, 0]                  # Home position

    try:
        while True:
            # Move to pose 1
            print("\nMoving to extended position")
            move_motors_to_positions(servos, pose1)
            if not wait_for_all_motors_idle(servos, TIMEOUT):
                print("Error: Motors did not reach position")
                for servo in servos:
                    emergency_stop(servo)
                break
            time.sleep(1)

            # Move to pose 2
            print("\nMoving to home position")
            move_motors_to_positions(servos, pose2)
            if not wait_for_all_motors_idle(servos, TIMEOUT):
                print("Error: Motors did not reach position")
                for servo in servos:
                    emergency_stop(servo)
                break
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        # Clean up
        notifier.stop()
        bus.shutdown()
        print("CAN bus shut down")

if __name__ == "__main__":
    main()
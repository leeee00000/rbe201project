#!/usr/bin/env python3


import serial
import time
import sys
import signal

class YahboomMotorDriver:
    """Control class for the four-channel encoder motor driver module."""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        """
        Initialize the serial port connection.
        
        Args:
            port: Serial device name
            baudrate: Baud rate
            timeout: Timeout duration
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connected = False
        self.motor_type = 1
        self.debug = True
        self.stop_sent = False  # Indicates whether a stop command has been sent
        
    def connect(self):
        """Establish the serial port connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.connected = True
            print(f"✓ Successfully connected to serial port {self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to open serial port: {e}")
            print("  Please check:")
            print("  1. Whether the USB cable is connected correctly")
            print("  2. Device permissions: sudo chmod 666 /dev/ttyUSB0")
            print("  3. Whether the device name is correct")
            return False
    
    def close(self):
        """Close the serial port connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("Serial port closed")
    
    def send_command(self, cmd, wait_response=True, max_retries=3, wait_time=0.15):
        """
        Send a command and receive the response.
        
        Args:
            cmd: Command content (excluding $ and #)
            wait_response: Whether to wait for a response
            max_retries: Maximum number of retries
            wait_time: Response wait time
        
        Returns:
            str: Response content; returns None on failure
        """
        if not self.connected:
            print("✗ Not connected to the serial port")
            return None
        
        full_cmd = f"${cmd}#"
        
        for attempt in range(max_retries):
            try:
                if self.debug:
                    print(f"  Sent: {full_cmd}")
                
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                self.ser.write(full_cmd.encode('utf-8'))
                self.ser.flush()
                
                if not wait_response:
                    return "ok"
                
                time.sleep(wait_time)
                
                responses = []
                start_time = time.time()
                
                while time.time() - start_time < 0.5:
                    if self.ser.in_waiting:
                        response = self.ser.readline().decode('utf-8').strip()
                        if response:
                            responses.append(response)
                            if self.debug:
                                print(f"  Received: {response}")
                            
                            if "ok" in response.lower() or "fail" in response.lower():
                                break
                    time.sleep(0.01)
                
                if responses:
                    return responses[-1]
                else:
                    if self.debug:
                        print("  ⚠ No response")
                    return None
                    
            except Exception as e:
                print(f"  ✗ Send failed (attempt {attempt+1}/{max_retries}): {e}")
                time.sleep(0.1)
        
        return None
    
    def emergency_stop(self):
        """
        Emergency stop for all motors with multiple safeguards.
        Send multiple stop commands to ensure that the motors stop.
        """
        print("\n>>> 🛑 Emergency stop for all motors...")
        
        # Try multiple stopping methods
        stop_commands = [
            b'$spd:0,0,0,0#',
            b'$pwm:0,0,0,0#',
            b'$stop#',
            b'$spd:0,0,0,0#',  # Send once more
        ]
        
        for i, cmd in enumerate(stop_commands):
            try:
                if self.ser and self.ser.is_open:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.ser.write(cmd)
                    self.ser.flush()
                    if self.debug:
                        print(f"  Stop command {i+1}: {cmd.decode().strip()}")
                    time.sleep(0.05)
            except Exception as e:
                if self.debug:
                    print(f"  Failed to send stop command {i+1}: {e}")
        
        self.stop_sent = True
        print("  ✅ Emergency stop completed")
    
    def set_motor_type(self, motor_type=1):
        """
        Set the motor type.
        """
        self.motor_type = motor_type
        print(f"\n--- Configuring motor type: {motor_type} ---")
        response = self.send_command(f"mtype:{motor_type}")
        return response
    
    def set_speed(self, m1, m2, m3, m4, wait_response=True):
        """
        Control the speeds of all four motors.
        """
        print(f"\n--- Setting speed: [{m1:4d}, {m2:4d}, {m3:4d}, {m4:4d}] ---")
        speeds = [max(-1000, min(1000, s)) for s in (m1, m2, m3, m4)]
        cmd = f"spd:{speeds[0]},{speeds[1]},{speeds[2]},{speeds[3]}"
        response = self.send_command(cmd, wait_response, wait_time=0.15)
        self.stop_sent = False  # Reset the flag
        return response
    
    def stop_all_motors(self):
        """Stop all motors by invoking the emergency stop."""
        self.emergency_stop()
        return True
    
    def read_encoders(self):
        """Read encoder data."""
        print("\n--- Reading encoder data ---")
        response = self.send_command("upload:1,0,0", wait_time=0.15)
        return response
    
    def test_single_motor(self, channel, speed=300, duration=3):
        """
        Test a single motor.
        """
        speeds = [0, 0, 0, 0]
        speeds[channel-1] = speed
        
        print(f"\n{'='*50}")
        print(f"Testing motor channel {channel} (speed: {speed}, duration: {duration} seconds)")
        print('='*50)
        
        self.set_speed(speeds[0], speeds[1], speeds[2], speeds[3])
        time.sleep(duration)
        self.stop_all_motors()
        time.sleep(0.3)
        self.read_encoders()


def signal_handler(signum, frame):
    """Signal handler that catches Ctrl+C."""
    print(f"\n\n⚠ Interrupt signal received ({signum}); stopping motors...")
    # The global variable driver is defined in main
    if 'driver' in globals() and driver:
        driver.emergency_stop()
    sys.exit(0)


def run_basic_test(driver):
    """Run the basic functionality test."""
    print("\n" + "="*50)
    print("Starting basic functionality test")
    print("="*50)
    
    driver.set_motor_type(motor_type=1)
    time.sleep(0.5)
    
    print("\n>>> All motors rotate forward at 50% speed for 3 seconds")
    driver.set_speed(500, 500, 500, 500)
    time.sleep(3)
    driver.read_encoders()
    
    print("\n>>> Stopping motors")
    driver.stop_all_motors()
    time.sleep(1)
    
    print("\n>>> All motors rotate in reverse at 30% speed for 3 seconds")
    driver.set_speed(-300, -300, -300, -300)
    time.sleep(3)
    driver.read_encoders()
    
    print("\n>>> Stopping motors")
    driver.stop_all_motors()
    time.sleep(1)
    
    print("\n>>> Incremental speed test")
    for speed in [100, 200, 300, 500, 700]:
        print(f"  Speed: {speed}")
        driver.set_speed(speed, speed, speed, speed)
        time.sleep(0.5)
    
    driver.stop_all_motors()


def run_individual_test(driver):
    """Test each motor individually."""
    print("\n" + "="*50)
    print("Starting individual motor test")
    print("="*50)
    
    driver.set_motor_type(motor_type=1)
    time.sleep(0.5)
    
    for ch in range(1, 5):
        driver.test_single_motor(ch, speed=400, duration=3)
        time.sleep(0.5)


def run_motor_pair_test(driver):
    """Run the motor pair test."""
    print("\n" + "="*50)
    print("Starting motor pair test")
    print("="*50)
    
    driver.set_motor_type(motor_type=1)
    time.sleep(0.5)
    
    print("\n>>> Pivot left: M1 and M3 forward, M2 and M4 reverse")
    driver.set_speed(400, -400, 400, -400)
    time.sleep(2)
    driver.stop_all_motors()
    time.sleep(0.5)
    
    print("\n>>> Moving forward: all motors rotate forward")
    driver.set_speed(300, 300, 300, 300)
    time.sleep(2)
    driver.stop_all_motors()
    time.sleep(0.5)
    
    driver.read_encoders()


def manual_control(driver):
    """Run manual control mode."""
    print("\n" + "="*50)
    print("Manual control mode")
    print("="*50)
    print("Command format: speed1 speed2 speed3 speed4")
    print("Speed range: -1000 to 1000")
    print("Enter 'q' to quit | 's' to stop all motors")
    print("Example: 500 0 -300 200")
    print("="*50)
    
    driver.set_motor_type(motor_type=1)
    time.sleep(0.5)
    
    while True:
        try:
            cmd = input("\nEnter four speed values: ").strip()
            
            if cmd.lower() == 'q':
                break
            elif cmd.lower() == 's':
                driver.stop_all_motors()
                continue
            
            parts = cmd.split()
            if len(parts) != 4:
                print("✗ Enter four speed values separated by spaces")
                continue
            
            speeds = [int(p) for p in parts]
            driver.set_speed(speeds[0], speeds[1], speeds[2], speeds[3])
            driver.read_encoders()
            
        except ValueError:
            print("✗ Enter valid integers")
        except KeyboardInterrupt:
            break


def main():
    """Main program."""
    global driver  # Declare globally for use by the signal handler
    
    print("="*50)
    print("Yahboom Four-Channel Encoder Motor Driver Control Program")
    print("Press Ctrl+C at any time to stop the motors")
    print("="*50)
    
    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create the driver object
    driver = YahboomMotorDriver(
        port='/dev/ttyUSB0',
        baudrate=115200,
        timeout=1
    )
    
    if not driver.connect():
        return
    
    try:
        print("\nSelect a test mode:")
        print("  1. Basic functionality test (all motors)")
        print("  2. Individual motor test")
        print("  3. Motor pair test (car mode)")
        print("  4. Manual control mode")
        
        try:
            choice = input("\nEnter an option (1-4): ").strip()
            mode = int(choice) if choice else 1
        except ValueError:
            mode = 1
        
        if mode == 1:
            run_basic_test(driver)
        elif mode == 2:
            run_individual_test(driver)
        elif mode == 3:
            run_motor_pair_test(driver)
        elif mode == 4:
            manual_control(driver)
        else:
            run_basic_test(driver)
            
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    except Exception as e:
        print(f"\n✗ Program runtime error: {e}")
    finally:
        # ✅ Always perform an emergency stop at the end
        try:
            driver.emergency_stop()
        except:
            pass
        
        time.sleep(0.3)
        driver.close()
        print("\nTest completed!")


if __name__ == "__main__":
    main()
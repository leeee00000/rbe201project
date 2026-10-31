#!/usr/bin/env python3


import csv
import datetime
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import warnings

import serial
import Hobot.GPIO as GPIO




SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200

TRACKING_PINS = {
    "X1": 15,  
    "X2": 18,  
    "X3": 16, 
    "X4": 11, 
}

BLACK_LEVEL = 0




MOTOR_TYPE = 1
MOTOR_DEADZONE = 1900

CONFIGURE_DEADZONE_ON_START = False

LEFT_DIRECTION = 1
RIGHT_DIRECTION = 1



BASE_PWM = 220

SOFT_INNER_PWM = 130
SOFT_OUTER_PWM = 220

MEDIUM_INNER_PWM = 0
MEDIUM_OUTER_PWM = 220


HARD_TURN_PWM = 210

CROSS_PWM = 180

LOST_REVERSE_PWM = 190

SEARCH_PWM = 200

MAX_PWM = 300



CONTROL_HZ = 50.0

RECENTER_BRAKE_SECONDS = 0.08

LOST_BRAKE_SECONDS = 0.06
LOST_REVERSE_SECONDS = 0.10

LOST_STOP_SECONDS = 5.0

COMMAND_REFRESH_INTERVAL = 0.25
DEBUG_INTERVAL = 0.5


DHT_BINARY_NAME = "dht11_rdk_x5_gpio401_server"

DHT_LOG_FILE_PREFIX = "temperature_points"
DHT_REQUEST_TIMEOUT_SECONDS = 10.0

DHT_CPU_CORE = 7
DHT_RT_PRIORITY = 50

MARKER_CONFIRM_SECONDS = 0.04

MARKER_CLEAR_SECONDS = 0.10


class MotorBoard:
    

    def __init__(self):
        self.ser = None
        self.connected = False

        self.lock = threading.RLock()

        self.last_command = None
        self.last_send_time = 0.0

    def connect(self):
        
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE,
                timeout=0.05,
                write_timeout=0.2,
            )

            time.sleep(0.15)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.connected = True

            print(f"✓ Successfully connected to the serial port {SERIAL_PORT}")
            return True

        except Exception as exc:
            print(f"✗ Serial port connection failed：{exc}")
            print(" Please verify the names and permissions of the USB and serial ports.")
            return False

    def send_raw(self, command):
       
        if not self.connected:
            return False

        if not self.ser or not self.ser.is_open:
            return False

        try:
            with self.lock:
                self.ser.write(
                    command.encode("utf-8")
                )
                self.ser.flush()

            return True

        except Exception as exc:
            print(f"\n✗ Serial port transmission failed：{exc}")
            return False

    def send_setting(
        self,
        command,
        description,
        timeout=0.8,
    ):
       
        if not self.connected:
            return False

        print(f"\n--- {description} ---")
        print(f"  send：{command}")

        try:
            with self.lock:
                self.ser.reset_input_buffer()

                self.ser.write(
                    command.encode("utf-8")
                )
                self.ser.flush()

                deadline = (
                    time.monotonic() + timeout
                )

                while time.monotonic() < deadline:
                    if self.ser.in_waiting > 0:
                        response = (
                            self.ser.readline()
                            .decode(
                                "utf-8",
                                errors="replace",
                            )
                            .strip()
                        )

                        if response:
                            print(f"  reception：{response}")

                            lower_response = (
                                response.lower()
                            )

                            if "ok" in lower_response:
                                return True

                            if "fail" in lower_response:
                                return False
                    else:
                        time.sleep(0.005)

            print("  ⚠ No response was received from the setting command.")
            return False

        except Exception as exc:
            print(f"  ✗ Setup failed：{exc}")
            return False

    def configure(self):
        
        type_ok = self.send_setting(
            f"$mtype:{MOTOR_TYPE}#",
            f"Configure motor type：{MOTOR_TYPE}",
        )

        if not type_ok:
            print("⚠ Motor type setting has not been confirmed successfully.")

        time.sleep(0.1)

        if CONFIGURE_DEADZONE_ON_START:
            deadzone_ok = self.send_setting(
                f"$deadzone:{MOTOR_DEADZONE}#",
               f"Configure motor dead zone: {MOTOR_DEADZONE}",
            )

            if not deadzone_ok:
                print("⚠ Motor dead zone setting failed to be confirmed successfully")

            time.sleep(0.1)
        else:
            print(
                f"✓ Use the saved dead zone of the driver board "
                f"{MOTOR_DEADZONE}"
            )

    @staticmethod
    def limit_pwm(value):
        
        return int(
            max(
                -MAX_PWM,
                min(MAX_PWM, round(value)),
            )
        )

    def set_pwm(
        self,
        m1,
        m2,
        m3,
        m4,
        force=False,
    ):
        
        values = tuple(
            self.limit_pwm(value)
            for value in (m1, m2, m3, m4)
        )

        now = time.monotonic()
        key = ("pwm", values)

        if (
            not force
            and key == self.last_command
            and (
                now - self.last_send_time
                < COMMAND_REFRESH_INTERVAL
            )
        ):
            return True

        command = (
            f"$pwm:{values[0]},{values[1]},"
            f"{values[2]},{values[3]}#"
        )

        result = self.send_raw(command)

        if result:
            self.last_command = key
            self.last_send_time = now

        return result

    def brake(self, force=False):
       
        now = time.monotonic()
        key = ("brake", (0, 0, 0, 0))

        if (
            not force
            and key == self.last_command
            and (
                now - self.last_send_time
                < COMMAND_REFRESH_INTERVAL
            )
        ):
            return True

        result = self.send_raw(
            "$spd:0,0,0,0#"
        )

        if result:
            self.last_command = key
            self.last_send_time = now

        return result

    def emergency_stop(self):
        
        if not self.ser or not self.ser.is_open:
            return False

        commands = [
            b"$spd:0,0,0,0#",
            b"$pwm:0,0,0,0#",
            b"$stop#",
            b"$spd:0,0,0,0#",
        ]

        print("\n>>> 🛑Immediately stop all the motors....")

        success_count = 0

        with self.lock:
            for index, command in enumerate(commands):
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()

                    self.ser.write(command)
                    self.ser.flush()

                    success_count += 1

                    print(
                        f"  stop instruction {index + 1}："
                        f"{command.decode('utf-8')}"
                    )

                    time.sleep(0.05)

                except Exception as exc:
                    print(
                        f"  stop instruction {index + 1} "
                        f"fail to send：{exc}"
                    )

        self.last_command = (
            "stop",
            (0, 0, 0, 0),
        )
        self.last_send_time = time.monotonic()

        if success_count == len(commands):
            print("  Stop commands sent; verify that the wheels have stopped.")
        else:
            print(
                f"  ⚠ Has sent "
                f"{success_count}/{len(commands)} "
               
            )

        return success_count > 0

    def close(self):
        
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port has been closed.")

        self.connected = False


class LogPersistenceError(RuntimeError):
    """The event log could not be committed; normal operation must stop."""


class DHTPointCollector:
    """Record marker events before requesting asynchronous sensor readings.

    Each event starts as ``pending``. A final status is committed at most once;
    late or duplicate replies cannot replace an already completed event.
    """

    LOG_FIELDS = (
        "RunID", "Detection point", "Trigger time", "Result time",
        "Humidity_%RH", "Temperature_°C", "Status", "Explanation",
    )

    def __init__(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.run_id = (
            datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            + "_" + uuid.uuid4().hex[:8]
        )
        self.binary_path = os.path.join(base_dir, DHT_BINARY_NAME)
        self.log_path = os.path.join(
            base_dir, f"{DHT_LOG_FILE_PREFIX}_{self.run_id}.csv"
        )
        self.process = None
        self.reader_thread = None
        self.write_lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.records = {}
        self.pending = {}  # point_id -> deadline from time.monotonic()
        self.stopping = False
        self.fatal_error = None
        self.log_prepared = False
        self.output_closed = False

    @staticmethod
    def timestamp():
        return datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")

    def _commit_locked(self, records):
        """Replace the CSV atomically, then publish the new in-memory state.

        The caller holds state_lock. If writing fails, the previous file and
        pending state remain intact; the main control loop receives a fault.
        """
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", newline="", encoding="utf-8-sig", delete=False,
                prefix=".temperature_points_", suffix=".tmp",
                dir=os.path.dirname(self.log_path),
            ) as csv_file:
                temporary_path = csv_file.name
                writer = csv.DictWriter(csv_file, fieldnames=self.LOG_FIELDS)
                writer.writeheader()
                writer.writerows(records.values())
                csv_file.flush()
                os.fsync(csv_file.fileno())
            # sudo owns the file; students must still be able to read/copy it.
            os.chmod(temporary_path, 0o644)
            os.replace(temporary_path, self.log_path)
            self.records = records
        except Exception as exc:
            self.fatal_error = f"Cannot save event log {self.log_path}: {exc}"
            print(f"\n✗ {self.fatal_error}. Stopping robot operation.", file=sys.stderr, flush=True)
            raise LogPersistenceError(self.fatal_error) from exc
        finally:
            if temporary_path is not None and os.path.exists(temporary_path):
                try:
                    os.unlink(temporary_path)
                except OSError:
                    pass

    def prepare_log(self):
        with self.state_lock:
            if self.log_prepared:
                return
            if os.path.exists(self.log_path):
                self.fatal_error = f"Refusing to overwrite an existing run log: {self.log_path}"
                raise LogPersistenceError(self.fatal_error)
            self._commit_locked({})
            self.log_prepared = True

    def start(self):
        if os.geteuid() != 0:
            raise RuntimeError(
                "Fixed-point temperature and humidity measurement requires root privileges. "
                "Please run this program using 'sudo python3'."
            )
        if not os.path.isfile(self.binary_path):
            raise RuntimeError(
                f"Cannot find the DHT11 acquisition program: {self.binary_path}. "
                "Please compile the C file according to the instructions."
            )
        if not os.access(self.binary_path, os.X_OK):
            raise RuntimeError(f"The DHT11 program is not executable: {self.binary_path}")
        self.prepare_log()
        self.process = subprocess.Popen(
            ["chrt", "-f", str(DHT_RT_PRIORITY), "taskset", "-c",
             str(DHT_CPU_CORE), self.binary_path, "--server"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
            text=True, bufsize=1,
        )
        time.sleep(0.15)
        if self.process.poll() is not None:
            raise RuntimeError(
                f"The DHT11 acquisition service failed to start: exit code {self.process.returncode}"
            )
        self.reader_thread = threading.Thread(
            target=self._read_results, name="dht11-result-reader", daemon=True,
        )
        self.reader_thread.start()
        print(f"✓ DHT11 service active: CPU {DHT_CPU_CORE}, real-time priority {DHT_RT_PRIORITY}")
        print(f"✓ RunID: {self.run_id}\n✓ Event log: {self.log_path}")

    def _finish_event(self, point_id, status, message="", humidity="", temperature=""):
        """Commit one terminal state, without discarding pending on failure."""
        point_id = str(point_id)
        with self.state_lock:
            existing = self.records.get(point_id)
            if existing is None or existing["Status"] != "pending":
                return False
            updated = dict(existing)
            updated.update({
                "Result time": self.timestamp(), "Humidity_%RH": humidity,
                "Temperature_°C": temperature, "Status": status, "Explanation": message,
            })
            records = dict(self.records)
            records[point_id] = updated
            self._commit_locked(records)
            self.pending.pop(point_id, None)
            return True

    def _finish_pending(self, status, message):
        with self.state_lock:
            unresolved = [point_id for point_id, row in self.records.items() if row["Status"] == "pending"]
            for point_id in unresolved:
                self._finish_event(point_id, status, message)

    def request(self, point_id):
        """Persist the trigger before writing READ to the subprocess pipe."""
        point_id = str(point_id)
        self.check_health()
        self.prepare_log()
        with self.state_lock:
            if point_id in self.records:
                raise ValueError(f"Duplicate detection point in this run: {point_id}")
            row = dict.fromkeys(self.LOG_FIELDS, "")
            row.update({
                "RunID": self.run_id, "Detection point": point_id,
                "Trigger time": self.timestamp(), "Status": "pending",
            })
            records = dict(self.records)
            records[point_id] = row
            self._commit_locked(records)
            self.pending[point_id] = time.monotonic() + DHT_REQUEST_TIMEOUT_SECONDS

        with self.write_lock:
            if self.stopping:
                self._finish_event(point_id, "cancelled", "Application is stopping")
                return False
            if (self.output_closed or self.process is None or self.process.poll() is not None
                    or self.process.stdin is None):
                self._finish_event(point_id, "request_failure", "DHT11 service is not running")
                return False
            try:
                self.process.stdin.write(f"READ {point_id}\n")
                self.process.stdin.flush()
            except (OSError, ValueError) as exc:
                self._finish_event(point_id, "request_failure", f"Cannot send READ: {exc}")
                return False
        print(f"\n📍 Detection point {point_id}: trigger recorded; asynchronous reading requested.", flush=True)
        return True

    def check_health(self):
        """Called by the control loop to surface disk faults and expire requests."""
        with self.state_lock:
            if self.fatal_error is not None:
                raise LogPersistenceError(self.fatal_error)
            now = time.monotonic()
            expired = [point_id for point_id, deadline in self.pending.items() if now >= deadline]
            for point_id in expired:
                self._finish_event(
                    point_id, "timeout", f"No result within {DHT_REQUEST_TIMEOUT_SECONDS:.1f} seconds"
                )

    def _handle_result(self, line):
        parts = line.split("\t", 4)
        if len(parts) < 2 or parts[0] != "DHT":
            if line:
                print(f"\n[DHT11] {line}", flush=True)
            return
        point_id = parts[1]
        with self.state_lock:
            if point_id not in self.pending:
                print(f"\n[DHT11] Ignoring late, duplicate or unknown reply for point {point_id}", flush=True)
                return
            if time.monotonic() >= self.pending[point_id]:
                self._finish_event(
                    point_id, "timeout", f"No result within {DHT_REQUEST_TIMEOUT_SECONDS:.1f} seconds"
                )
                return
            if len(parts) == 5 and parts[2] == "OK":
                try:
                    humidity, temperature = float(parts[3]), float(parts[4])
                    if not (math.isfinite(humidity) and math.isfinite(temperature)):
                        raise ValueError("non-finite sensor value")
                except ValueError:
                    self._finish_event(point_id, "sensor_failure", f"Invalid sensor result: {line}")
                    return
                if self._finish_event(
                    point_id, "success", humidity=f"{humidity:.1f}", temperature=f"{temperature:.1f}"
                ):
                    print(
                        f"\n🌡 Detection point {point_id}: humidity {humidity:.1f}%RH, "
                        f"temperature {temperature:.1f}°C. Saved", flush=True,
                    )
            else:
                message = parts[3] if len(parts) >= 4 and parts[2] == "ERROR" else f"Invalid sensor result: {line}"
                self._finish_event(point_id, "sensor_failure", message)
                print(f"\n⚠ Detection point {point_id}: sensor_failure: {message}", flush=True)

    def _read_results(self):
        if self.process is None or self.process.stdout is None:
            return
        try:
            for raw_line in self.process.stdout:
                self._handle_result(raw_line.rstrip("\r\n"))
            # Synchronize EOF with request writes, so a broken READ can first
            # receive its more specific request_failure status.
            with self.write_lock:
                self.output_closed = True
                if not self.stopping:
                    self._finish_pending("sensor_failure", "DHT11 service closed its output before returning a result")
        except LogPersistenceError:
            # check_health() raises the stored fault on the next control tick.
            return
        except (OSError, ValueError, UnicodeError) as exc:
            self.output_closed = True
            try:
                if not self.stopping:
                    self._finish_pending("sensor_failure", f"Cannot read DHT11 output: {exc}")
            except LogPersistenceError:
                pass

    def stop(self):
        """Cancel unresolved events before shutting down the sensor service."""
        with self.state_lock:
            self.stopping = True
        persistence_error = None
        try:
            self._finish_pending("cancelled", "Application stopped before receiving a result")
        except LogPersistenceError as exc:
            persistence_error = exc
        try:
            if self.process is not None:
                if self.process.poll() is None and self.process.stdin is not None:
                    try:
                        with self.write_lock:
                            self.process.stdin.write("QUIT\n")
                            self.process.stdin.flush()
                    except (OSError, ValueError):
                        pass
                try:
                    self.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=1.0)
                if self.reader_thread is not None and self.reader_thread.is_alive():
                    self.reader_thread.join(timeout=1.0)
                if self.process.stdin is not None:
                    self.process.stdin.close()
                if self.process.stdout is not None:
                    self.process.stdout.close()
                self.process = None
                print("DHT11 acquisition service stopped.")
        finally:
            if persistence_error is not None:
                raise persistence_error


class LineFollower:
    

    SENSOR_WEIGHTS = {
        "X1": -3.0,
        "X2": -1.0,
        "X3": 1.0,
        "X4": 3.0,
    }

    def __init__(self):
        self.motor = MotorBoard()
        self.dht = DHTPointCollector()

        self.running = True
        self.shutting_down = False
        self.gpio_ready = False

        self.current_left = 0
        self.current_right = 0
        self.current_mode = "PWM"
        self.fault_reason = None

        self.last_error = 0.0
        self.lost_direction = 0
        self.lost_since = None

        self.centered = False
        self.center_brake_until = None
        self.ever_moved = False

        self.last_sensor_signature = None
        self.last_debug_time = 0.0

        self.marker_count = 0
        self.marker_black_since = None
        self.marker_clear_since = None
        self.marker_latched = False

        signal.signal(
            signal.SIGINT,
            self._stop_signal_handler,
        )
        signal.signal(
            signal.SIGTERM,
            self._stop_signal_handler,
        )
        signal.signal(
            signal.SIGTSTP,
            self._suspend_handler,
        )

    def setup(self):
        
        if not self.motor.connect():
            raise RuntimeError("Motor drive connection failed")

        GPIO.setmode(GPIO.BOARD)

        for name, pin in TRACKING_PINS.items():
            GPIO.setup(pin, GPIO.IN)
            print(f"✓ {name}：Physical pin {pin}")

        self.gpio_ready = True

        self.motor.configure()
        self.dht.start()

    def read_sensors(self):
        
        return {
            name: int(GPIO.input(pin))
            for name, pin in TRACKING_PINS.items()
        }

    def calculate_error(self, sensors):
        
        active = [
            name
            for name, value in sensors.items()
            if value == BLACK_LEVEL
        ]

        if not active:
            return None, 0

        error = (
            sum(
                self.SENSOR_WEIGHTS[name]
                for name in active
            )
            / len(active)
        )

        return error, len(active)

    def detect_temperature_marker(
        self,
        sensors,
        now,
    ):
       
        all_black = all(
            value == BLACK_LEVEL
            for value in sensors.values()
        )

        if all_black:
            self.marker_clear_since = None

            if self.marker_latched:
                return None

            if self.marker_black_since is None:
                self.marker_black_since = now
                return None

            if (
                now - self.marker_black_since
                < MARKER_CONFIRM_SECONDS
            ):
                return None

            self.marker_latched = True
            self.marker_black_since = None
            self.marker_count += 1

            requested = self.dht.request(
                self.marker_count
            )

            if requested:
                return (
                    f"Checkpoint{self.marker_count}，"
                    "Temperature and humidity collection has been triggered."
                )

            return (
                f"check point{self.marker_count}，"
                "Temperature and humidity triggering failed"
            )

        self.marker_black_since = None

        if not self.marker_latched:
            self.marker_clear_since = None
            return None

        if self.marker_clear_since is None:
            self.marker_clear_since = now
            return None

        if (
            now - self.marker_clear_since
            >= MARKER_CLEAR_SECONDS
        ):
            self.marker_latched = False
            self.marker_clear_since = None

        return None

    def drive_pwm(self, left, right):
       
        if self.shutting_down:
            return

        requested_left = int(
            max(
                -MAX_PWM,
                min(MAX_PWM, round(left)),
            )
        )

        requested_right = int(
            max(
                -MAX_PWM,
                min(MAX_PWM, round(right)),
            )
        )

        left_output = int(
            requested_left * LEFT_DIRECTION
        )

        right_output = int(
            requested_right * RIGHT_DIRECTION
        )

       
        sent = self.motor.set_pwm(
            left_output,
            left_output,
            right_output,
            right_output,
        )
        if not sent:
            self._motor_fault("PWM command could not be sent")
        self.current_left = requested_left
        self.current_right = requested_right
        self.current_mode = "PWM"
        if requested_left != 0 or requested_right != 0:
            self.ever_moved = True

    def _motor_fault(self, reason):
        """Latch a communication fault; no normal movement may follow."""
        self.running = False
        self.shutting_down = True
        self.current_mode = "fault"
        self.fault_reason = reason
        print(f"\n✗ Motor fault: {reason}. Attempting an emergency stop.", file=sys.stderr, flush=True)
        try:
            if not self.motor.emergency_stop():
                print("⚠ Stop commands could not be sent; disconnect motor power.", file=sys.stderr, flush=True)
        except Exception as exc:
            print(f"⚠ Emergency-stop transmission failed: {exc}", file=sys.stderr, flush=True)
        raise RuntimeError(reason)

    def brake(self):
        
        if self.shutting_down:
            return

        if not self.motor.brake():
            self._motor_fault("Brake command could not be sent")
        self.current_left = 0
        self.current_right = 0
        self.current_mode = "braking"


    def control_center(self, now):
       
        self.lost_since = None

      
        if not self.ever_moved:
            self.centered = True
            self.center_brake_until = None

            self.drive_pwm(
                BASE_PWM,
                BASE_PWM,
            )

            return "Centered during startup, fixed PWM straight drive"

        
        if not self.centered:
            if self.center_brake_until is None:
                self.center_brake_until = (
                    now + RECENTER_BRAKE_SECONDS
                )

            if now < self.center_brake_until:
                self.brake()

                remaining = (
                    self.center_brake_until - now
                )

                return (
                    "Re-center, activate the emergency brake."
                    f"residue {remaining:.3f}s"
                )

            self.centered = True
            self.center_brake_until = None

        self.drive_pwm(
            BASE_PWM,
            BASE_PWM,
        )

        return "Black line is centered, fixed PWM straight driving"

    def control_visible_line(
        self,
        error,
        black_count,
        now,
    ):
        
        self.lost_since = None

    
        if black_count == 4:
            self.centered = False
            self.center_brake_until = None

            self.drive_pwm(
                CROSS_PWM,
                CROSS_PWM,
            )

            return "Four channels are all black, fixed PWM low-speed straight driving"

        absolute_error = abs(error)


        if absolute_error <= 0.25:
            return self.control_center(now)

        self.centered = False
        self.center_brake_until = None

        self.last_error = error

        if error < 0:
            self.lost_direction = -1
        elif error > 0:
            self.lost_direction = 1

       
        if absolute_error <= 1.25:
            if error < 0:
                self.drive_pwm(
                    SOFT_INNER_PWM,
                    SOFT_OUTER_PWM,
                )

                return (
                    f"Slightly to the left, error {error:+.2f}"
                )

            self.drive_pwm(
                SOFT_OUTER_PWM,
                SOFT_INNER_PWM,
            )

            return (
                f"Slightly to the right, error {error:+.2f}"
            )

        
        if absolute_error <= 2.25:
            if error < 0:
                self.drive_pwm(
                    MEDIUM_INNER_PWM,
                    MEDIUM_OUTER_PWM,
                )

                return (
                    f"Clearly leaning to the left, error {error:+.2f}"
                )

            self.drive_pwm(
                MEDIUM_OUTER_PWM,
                MEDIUM_INNER_PWM,
            )

            return (
                f"Clearly leaning to the right, error {error:+.2f}"
            )

       
        if error < 0:
            self.drive_pwm(
                -HARD_TURN_PWM,
                HARD_TURN_PWM,
            )

            return (
                f"Extremely left-leaning, PWM makes a left turn while stationary，"
                f"error {error:+.2f}"
            )

        self.drive_pwm(
            HARD_TURN_PWM,
            -HARD_TURN_PWM,
        )

        return (
            f"Extremely right-leaning, PWM rotates to the right in place，"
            f"error {error:+.2f}"
        )

    def control_lost_line(self, now):
       
        self.centered = False
        self.center_brake_until = None

        if self.lost_since is None:
            self.lost_since = now

            if self.last_error < 0:
                self.lost_direction = -1
            elif self.last_error > 0:
                self.lost_direction = 1

        elapsed = now - self.lost_since

        if elapsed >= LOST_STOP_SECONDS:
            self.motor.emergency_stop()

            self.current_left = 0
            self.current_right = 0
            self.running = False

            return (
                f"Drop the thread {elapsed:.1f}s，"
                "Stop and exit"
            )

       
        if elapsed < LOST_BRAKE_SECONDS:
            self.brake()

            return (
                f"Drop the thread {elapsed:.3f}s，"
                "Active Braking"
            )

        reverse_end = (
            LOST_BRAKE_SECONDS
            + LOST_REVERSE_SECONDS
        )

        
        if elapsed < reverse_end:
            self.drive_pwm(
                -LOST_REVERSE_PWM,
                -LOST_REVERSE_PWM,
            )

            return (
                f"Drop the thread {elapsed:.3f}s，"
                "PWM temporary setback"
            )

       
        if self.lost_direction < 0:
            self.drive_pwm(
                -SEARCH_PWM,
                SEARCH_PWM,
            )

            return (
                f"Drop the thread {elapsed:.2f}s，"
                "向左PWM搜索"
            )

        if self.lost_direction > 0:
            self.drive_pwm(
                SEARCH_PWM,
                -SEARCH_PWM,
            )

            return (
                f"Drop the thread {elapsed:.2f}s，"
                "向右PWM搜索"
            )

        self.drive_pwm(
            -SEARCH_PWM,
            SEARCH_PWM,
        )

        return (
            f"Drop the thread {elapsed:.2f}s，"
            "Default leftward search"
        )

    def print_debug(
        self,
        now,
        sensors,
        state,
    ):
       
        signature = tuple(
            sensors[name]
            for name in ("X1", "X2", "X3", "X4")
        )

        sensors_changed = (
            signature != self.last_sensor_signature
        )

        heartbeat_due = (
            now - self.last_debug_time
            >= DEBUG_INTERVAL
        )

        if not sensors_changed and not heartbeat_due:
            return

        sensor_text = " ".join(
            f"{name}:{value}"
            for name, value in sensors.items()
        )

        reason = (
           "Sensor change"
            if sensors_changed
            else "Status"
        )

        print(
            f"{sensor_text} | "
            f"{state} | "
            f"{self.current_mode} "
            f"{self.current_left:+d}/"
            f"{self.current_right:+d} | "
            f"{reason}",
            flush=True,
        )

        self.last_sensor_signature = signature
        self.last_debug_time = now

    def cleanup_gpio(self):
       
        if not self.gpio_ready:
            return

        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="No channels have been set up.*",
                    category=RuntimeWarning,
                )

                GPIO.cleanup()

        except Exception as exc:
            print(f"\nGPIO Clearing Warning：{exc}")

        finally:
            self.gpio_ready = False

    def run(self):
        
        period = 1.0 / CONTROL_HZ

        print("\nFour-channel PWM constant-speed patrol line operation has been initiated. Press Ctrl+C to stop.")
        print(
            f"control frequency：{CONTROL_HZ:.0f} Hz；"
            f"Basic PWM：{BASE_PWM}；"
            f"dead zone：{MOTOR_DEADZONE}"
        )
        print(
            "On the left side of M1/M2, on the right side of M3/M4;" 
            "Movement uses PWM, braking uses SPD = 0"
        )
        print(
            "Fixed-point collection: Four channels remain completely black continuously"
            f"{MARKER_CONFIRM_SECONDS:.2f}s "
            "Trigger a DHT11, and the car will not stop."
        )

        try:
            self.setup()

            next_tick = time.monotonic()

            while self.running:
                self.dht.check_health()
                now = time.monotonic()
                sensors = self.read_sensors()

                error, black_count = (
                    self.calculate_error(sensors)
                )

                marker_state = (
                    self.detect_temperature_marker(
                        sensors,
                        now,
                    )
                )

                if error is None:
                    state = self.control_lost_line(
                        now
                    )
                else:
                    state = (
                        self.control_visible_line(
                            error,
                            black_count,
                            now,
                        )
                    )

                if marker_state is not None:
                    state = (
                        f"{state}；{marker_state}"
                    )

                self.print_debug(
                    now,
                    sensors,
                    state,
                )

                if not self.running:
                    break

                next_tick += period
                delay = next_tick - time.monotonic()

                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

        except KeyboardInterrupt:
            pass

        except Exception as exc:
            self.fault_reason = str(exc)
            self.current_mode = "fault"
            self.running = False
            print(
                f"\n run-time error：{exc}",
                file=sys.stderr,
            )

        finally:
            self.running = False
            self.shutting_down = True

            signal.signal(
                signal.SIGINT,
                signal.SIG_IGN,
            )
            signal.signal(
                signal.SIGTERM,
                signal.SIG_IGN,
            )

            print("\n Carry out the second parking insurance before exiting")
            try:
                self.motor.emergency_stop()
            except Exception as exc:
                self.fault_reason = str(exc)
                print(f"\n✗ Emergency-stop error: {exc}", file=sys.stderr, flush=True)
            try:
                self.dht.stop()
            except Exception as exc:
                self.fault_reason = str(exc)
                print(f"\n✗ DHT11 shutdown error: {exc}", file=sys.stderr, flush=True)
            finally:
                try:
                    time.sleep(0.1)
                    self.motor.close()
                finally:
                    self.cleanup_gpio()

            print("\nStop commands attempted; serial and GPIO cleanup completed.")
        return 1 if self.fault_reason is not None else 0

    def _stop_signal_handler(
        self,
        signum,
        frame,
    ):
        
        if self.shutting_down:
            return

        self.shutting_down = True
        self.running = False

        signal.signal(
            signal.SIGINT,
            signal.SIG_IGN,
        )
        signal.signal(
            signal.SIGTERM,
            signal.SIG_IGN,
        )

        print("\nReceive the stop signal, immediately stop the vehicle.")

        self.motor.emergency_stop()

        raise KeyboardInterrupt

    def _suspend_handler(
        self,
        signum,
        frame,
    ):
        
        if self.shutting_down:
            return

        self.shutting_down = True
        self.running = False

        print("\nReceived the stop signal. After stopping, the operation was suspended.")

        self.motor.emergency_stop()

        signal.signal(
            signal.SIGTSTP,
            signal.SIG_DFL,
        )

        os.kill(
            os.getpid(),
            signal.SIGTSTP,
        )

        raise KeyboardInterrupt


if __name__ == "__main__":
    sys.exit(LineFollower().run())

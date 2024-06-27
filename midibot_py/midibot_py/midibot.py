import threading
import json
import time
import math
from .serial_comm import SerialCommunication
from .socket_comm import SocketCommunication
from .sensors import Sensors
from .default_configs import default_config


class DifferentialDriveRobot:
    """
    Singleton class to manage communication and control of a differential drive robot.
    Supports both serial and socket communication.
    """

    _instance = None  # Class-level attribute to hold the single instance
    _lock = threading.Lock()  # Class-level lock to ensure thread safety

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # First check (without lock)
            with cls._lock:  # Acquire lock to enter critical section
                if cls._instance is None:  # Double-check (with lock)
                    cls._instance = super(DifferentialDriveRobot, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self, serial_port, baudrate=115200, timeout=1, ip="192.168.137.28", socket_port=8080, config_file=None, stepper_ids=[1,2], u_ids=[1,2,3,4], b_ids=[1,2,3,4], imu_connected=False, u_median_filter_len=3, default_emergency_behavior = False):
        """
        Initialize the robot with connection parameters and setup communication interfaces.

        Args:
            serial_port (str): Serial port on pc or raspberry pi for connecting to the robot.
            baudrate (int, optional): Baud rate for serial communication. Defaults to 115200.
            timeout (int, optional): Timeout in seconds for communication operations. Defaults to 1.
            ip (str, optional): IP address of pico for socket communication. Defaults to "192.168.137.28".
            socket_port (int, optional): Port for socket communication on pc raspberry pi. Defaults to 8080.
            stepper_ids ([int], optional): Integer array of stepper motor IDs. Defaults to [1, 2].
            u_ids ([int], optional): Integer array of ultrasonic sensors. Defaults to [1, 2, 3, 4].
            b_ids ([int], optional): Integer array of bumper sensors. Defaults to [1, 2, 3, 4].
            imu_connected (bool, optional): Whether IMU is connected. Defaults to False.
            u_median_filter_len (int, optional): Length of median filter window. Defaults to 3.
            default_emergency_behavior (bool, optional): Default behavior of bumper switches coded on pico. Defaults to True.
                        Change to False if custom behavior is desired. Bumper switches are be polled for data if enabled.
        """
        if self._initialized:
            return

        # Initialize connection parameters provided by user
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ip = ip
        self.socket_port = socket_port
        self.timeout = timeout
        self.config_file = config_file
        self.configs = None

        # Initialize flags for connection states
        self.serial_running = False
        self.socket_running = False

        # If not config file, initialize sensor as provided in the arguments else, read from config file
        if not config_file:
            self.stepper_ids = stepper_ids
            self.u_ids = u_ids
            self.b_ids = b_ids
            self.imu_connected = imu_connected
            self.default_emergency_behavior = default_emergency_behavior
        else:
            self.u_ids, self.b_ids, self.imu_connected, self.stepper_ids, self.default_emergency_behavior = self.read_config_file(config_file)

        # Initialize communication and sensor objects
        self.sensors = Sensors(stepper_ids, u_ids, b_ids, imu_connected, u_median_filter_len)
        self.serial_comm = SerialCommunication(serial_port, baudrate, timeout)
        self.socket_comm = SocketCommunication(ip, socket_port)

        # Initialize internal thread lock
        self._internal_lock = threading.Lock()
        self._initialized = True

        # Robot parameters
        self._wheel_radius = 0.045  # in meters
        self._wheel_separation = 0.295  # in meters
        self._ticks_per_rev = 600  # Number of ticks per revolution of the motor

    @property
    def wheel_radius(self):
        return self._wheel_radius

    @wheel_radius.setter
    def wheel_radius(self, value):
        if value > 0:
            self._wheel_radius = value
        else:
            raise ValueError("Wheel radius must be a positive value")

    @property
    def wheel_separation(self):
        return self._wheel_separation

    @wheel_separation.setter
    def wheel_separation(self, value):
        if value > 0:
            self._wheel_separation = value
        else:
            raise ValueError("Wheel separation must be a positive value")

    @property
    def ticks_per_rev(self):
        return self._ticks_per_rev

    @ticks_per_rev.setter
    def ticks_per_rev(self, value):
        if value > 0:
            self._ticks_per_rev = value
        else:
            raise ValueError("Ticks per revolution must be a positive value")

    def connect(self, connection_type=None):
        """
        Establish a connection to the robot using either serial or socket communication.
        Tries to connect using serial first, and falls back to socket if unsuccessful.

        Args:
            connection_type (str, optional): Type of connection ('serial' or 'socket'). Defaults to None.
        """
        if connection_type == "serial":
            self._connect_serial()
        elif connection_type == "socket":
            self._connect_socket()
        else:
            for _ in range(3):
                self._connect_serial()
                if self.serial_running:
                    break
            else:
                if not self.serial_running:
                    self._connect_socket()        

    def _connect_serial(self):
        """
        Attempt to establish a serial connection.
        """
        
        self.serial_comm.connect()
        self.serial_running = self.serial_comm.is_connected()
        self.pico_config()


    def _connect_socket(self):
        """
        Attempt to establish a socket connection.
        """
        try:
            self.socket_comm.connect()
            self.socket_running = self.socket_comm.is_connected()
            self.pico_config()
        except Exception as e:
            print("Socket connection error:", e)

    def disconnect(self):
        """
        Disconnect both serial and socket connections.
        """
        if self.serial_running:
            self.serial_comm.disconnect()
            self.serial_running = False
        if self.socket_running:
            self.socket_comm.disconnect()
            self.socket_running = False
    
    def pico_config(self):
        #Establish handshake and send configuration to Pico
        self.send_command("HANDSHAKE")
        
        while not self.sensors.handshake:
            print('User waiting for handshake...')
            time.sleep(0.5)

        if not self.configs:
            config = default_config

            # Update the configuration based on user input
            if "SENSORS" in config:
                if "ultrasonic" in config["SENSORS"]:
                    for sensor in config["SENSORS"]["ultrasonic"]["sensors"]:
                        if sensor["id"] in self.u_ids:
                            sensor["enabled"] = True
                if "bumper_switches" in config["SENSORS"]:
                    for sensor in config["SENSORS"]["bumper_switches"]["sensors"]:
                        if sensor["id"] in self.b_ids:
                            sensor["enabled"] = True
                if "imu" in config["SENSORS"]:
                    config["SENSORS"]["imu"]["enabled"] = self.imu_connected

            if "stepper_motors" in config:
                for motor in config["stepper_motors"]:
                    if motor["id"] in self.stepper_ids:
                        motor["enabled"] = True

            config["default_emergency_behavior"] = self.default_emergency_behavior

            self.send_command(json.dumps(config))
        else:
            self.send_command(json.dumps(self.configs))
    
    def read_config_file(self, config_file):
            """
            Read a config.json file and return arrays of enabled sensor IDs and polling rates.

            Args:
                config_file (str): Path to the config.json file.

            Returns: 
                ultrasonic_ids (list): List of enabled ultrasonic sensor IDs.
                bumper_switch_ids (list): List of enabled bumper switch IDs.
                imu_enabled (bool): Flag indicating whether IMU is enabled.
                stepper_ids (list): List of enabled stepper motor IDs.
                default_emergency_behavior (bool): Flag indicating whether default bumper behavior is enabled.

            """
            try:
                with open(config_file, 'r') as f:
                    config_data = json.load(f)
                    self.configs = config_data
            except FileNotFoundError:
                print(f"Error: Config file '{config_file}' not found")
                return None
            except json.JSONDecodeError:
                print(f"Error: Unable to parse JSON from config file '{config_file}'")
                return None

            ultrasonic_ids = []
            bumper_switch_ids = []
            imu_enabled = False
            stepper_ids = []
            default_emergency_behavior = False


            # Read ultrasonic sensor IDs
            if "SENSORS" in config_data and "ultrasonic" in config_data["SENSORS"]:
                for sensor in config_data["SENSORS"]["ultrasonic"]["sensors"]:
                    if sensor.get("enabled", False):
                        ultrasonic_ids.append(sensor["id"])

            # Read bumper switch IDs
            if "SENSORS" in config_data and "bumper_switches" in config_data["SENSORS"]:
                for sensor in config_data["SENSORS"]["bumper_switches"]["sensors"]:
                    if sensor.get("enabled", False):
                        bumper_switch_ids.append(sensor["id"])

            # Read IMU enabled status
            if "SENSORS" in config_data and "imu" in config_data["SENSORS"]:
                imu_enabled = config_data["SENSORS"]["imu"].get("enabled", False)

            # Read stepper motor IDs
            if "stepper_motors" in config_data:
                for motor in config_data["stepper_motors"]:
                    if motor.get("enabled", False):
                        stepper_ids.append(motor["id"])

            # Read enable default bumper behavior flag
            default_emergency_behavior = config_data.get("default_emergency_behavior", False)

            return ultrasonic_ids, bumper_switch_ids, imu_enabled, stepper_ids, default_emergency_behavior

    def set_data_callback(self, callback):
        """
        Set a callback function to handle incoming data from the robot.

        Args:
            callback (function): Callback function to handle incoming data.
        """
        if self.serial_running:
            self.serial_comm.set_data_callback(callback)
        elif self.socket_running:
            self.socket_comm.set_data_callback(callback)

    def send_command(self, command):
        """
        Send a command to the robot using either serial or socket communication.

        Args:
            command (str): Command to be sent to the robot.
        """
        if self.serial_running:
            self.serial_comm.send_command(command)
        if self.socket_running:
            self.socket_comm.send_command(command)

    def set_ticks_duration(self, left_ticks, right_ticks, duration_l, duration_r):
        """
        Set the number of ticks and duration for the left and right wheels of the robot.

        Args:
            left_ticks (int): Number of ticks for the left wheel.
            right_ticks (int): Number of ticks for the right wheel.
            duration_l (float): Duration to reach the left ticks in seconds.
            duration_r (float): Duration to reach the right ticks in seconds.
        """
        command_l = f"t l {left_ticks} {duration_l}"
        command_r = f"t r {right_ticks} {duration_r}"
        self.send_command(command_l)
        self.send_command(command_r)
    
    def set_speed(self, left_speed, right_speed):
        """
        Set the speed of the left and right wheels of the robot.

        Args:
            left_speed (float): Speed of the left wheel. 
            right_speed (float): Speed of the right wheel.
        """
        command_l = f"s l {left_speed}"
        command_r = f"s r {right_speed}"
        self.send_command(command_l)
        self.send_command(command_r)

    def move_forward(self, duration=1, speed=500):
        """
        Move the robot forward for a specified duration.

        Args:
            duration (float): Duration to move forward in seconds.
            speed (int): Speed at which to move forward. Defaults to 500.
                                minimum = 100 (ideally min=250), maximum = 4000 (ideally max=3600)
        """
        self.set_speed(speed, speed)
        time.sleep(duration)
        self.stop()
    
    def rotate(self, duration=5, speed=600, direction="cw"):
        """
        Rotate the robot at a given speed in a give direction for a set duration.

        Args:
            duration (float): Duration of rotation in seconds. Defaults to 5 s
            speed (int): Speed of rotation in frequency of wheel rotation. Defaults to 600 Hz
            direction (string): 'cw' for clockwise roation. 'ccw' for counter-clockwise rotation
        
        """
        if direction.lower() == "cw":
            self.set_speed(speed, -speed)
        elif direction.lower() == "ccw":
            self.set_speed(-speed, speed)
        else:
            print("Set direction is incorrect. direction should be 'cw' or 'ccw'.")
            return
        time.sleep(duration)
        self.stop()

    def send_twist(self, linear_speed, angular_speed):
        """
        Convert desired linear and angular speeds to wheel frequencies and set the robot speed.

        Args:
            linear_speed (float): Desired linear speed in meters per second.
            angular_speed (float): Desired angular speed in radians per second.
        """
        w_l = (linear_speed - (angular_speed * self._wheel_separation / 2)) / self._wheel_radius
        w_r = (linear_speed + (angular_speed * self._wheel_separation / 2)) / self._wheel_radius

        f_l = w_l / (2 * math.pi) * self._ticks_per_rev
        f_r = w_r / (2 * math.pi) * self._ticks_per_rev

        f_l = round(f_l, 3)
        f_r = round(f_r, 3)

        self.set_speed(f_l, f_r)

    def stop(self):
        """
        Stop the robot by setting both wheel speeds to zero.
        """
        self.set_speed(0, 0)

    def get_sensor_data(self, sensor_type="u"):
        """
        Get sensor data from the robot. Default is ultrasound sensor data.
        """
        return self.sensors.request_sensor_data(sensor_type)

    def get_status(self):
        """
        Request the current status of the robot.
        """
        self.send_command("GET_STATUS")

    def __del__(self):
        """
        Destructor to stop the robot and disconnect on deletion.
        """
        self.stop()
        self.disconnect()

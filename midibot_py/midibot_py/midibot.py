import threading
import json
import time
import math
from .serial_comm import SerialCommunication
from .socket_comm import SocketCommunication
from .sensors import Sensors


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

    def __init__(self, serial_port, baudrate=115200, timeout=1, ip="192.168.137.28", socket_port=8080, stepper_ids=[1,2], u_ids=[1,2,3,4], b_ids=[1,2,3,4], imu_connected=False, u_moving_avg_len=3):
        """
        Initialize the robot with connection parameters and setup communication interfaces.

        Args:
            serial_port (str): Serial port for connecting to the robot.
            baudrate (int, optional): Baud rate for serial communication. Defaults to 9600.
            timeout (int, optional): Timeout for communication operations. Defaults to 1.
            ip (str, optional): IP address for socket communication. Defaults to "192.168.137.28".
            socket_port (int, optional): Port for socket communication. Defaults to 8080.
            u_count (int, optional): Number of ultrasonic sensors. Defaults to 4.
            b_count (int, optional): Number of bump sensors. Defaults to 4.
            imu_connected (bool, optional): Whether IMU is connected. Defaults to False.
            u_moving_avg_len (int, optional): Length of moving average for ultrasonic sensors. Defaults to 3.
        """
        if self._initialized:
            return

        # Initialize connection parameters provided by user
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ip = ip
        self.socket_port = socket_port
        self.timeout = timeout
        self.stepper_ids = stepper_ids
        self.u_ids = u_ids
        self.b_ids = b_ids
        self.imu_connected = imu_connected
        self.u_moving_avg_len = u_moving_avg_len

        # Initialize communication and sensor objects
        self.sensors = Sensors(stepper_ids, u_ids, b_ids, imu_connected, u_moving_avg_len)
        self.serial_comm = SerialCommunication(serial_port, baudrate, timeout)
        self.socket_comm = SocketCommunication(ip, socket_port)
        
        # Initialize flags for connection states
        self.serial_running = False
        self.socket_running = False

        # Initialize internal thread lock
        self._internal_lock = threading.Lock()
        self._initialized = True

        # Robot parameters (should be read only)
        self._wheel_radius = 0.045  # in meters
        self._wheel_separation = 0.295  # in meters
        self._ticks_per_rev = 600  # Number of ticks per revolution of the motor

        # Current speeds
        self.left_speed = 0
        self.right_speed = 0
        self.angular_speed = 0

        
        

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
    
    def pico_config(self):
        #Establish handshake and send configuration to Pico
        self.send_command("HANDSHAKE")
        
        while not self.sensors.handshake:
            print('User waiting for handshake...')
            time.sleep(0.5)
            
        config = {
            "sensors": {
                "ultrasonic": [{"id": i, "enabled": True} for i in self.u_ids],
                "bumper_switches": [{"id": i, "enabled": True} for i in self.b_ids],
                "imu": {"enabled": self.imu_connected}
            },
            "stepper_motors": [{"id": i, "enabled": True} for i in self.stepper_ids]
        }
        self.send_command(json.dumps(config))

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

    def move_forward(self, duration=1, speed=500 ):
        """
        Move the robot forward for a specified duration.

        Args:
            duration (float): Duration to move forward in seconds.
        """
        self.set_speed(speed, speed)  # Example values, adjust as needed
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
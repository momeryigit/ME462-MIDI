import threading
import time
from collections import deque
from .serial_comm import SerialCommunication
from .socket_comm import SocketCommunication
from .sensors import Sensors
import math


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

    def __init__(
        self, serial_port, baudrate=9600, timeout=1, ip="192.168.137.28", socket_port=8080, u_count=4, b_count=4, imu_connected=False, u_moving_avg_len=3
    ):
        if self._initialized:
            return

        # Initialize connection parameters provided by user
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ip = ip
        self.socket_port = socket_port
        self.timeout = timeout
        self.u_count = u_count
        self.b_count = b_count
        self.imu_connected = imu_connected
        self.u_moving_avg_len = u_moving_avg_len

        self.sensors = Sensors(u_count, b_count, imu_connected, u_moving_avg_len)
        self.serial_comm = SerialCommunication(self.serial_port, self.baudrate, self.timeout)
        self.socket_comm = SocketCommunication(self.ip, self.socket_port)
        self.serial_running = False
        self.socket_running = False
        
        self.raw_data_queue = None
        self.data_callback = None

        self._internal_lock = threading.Lock()
        self._initialized = True

        # Robot parameters
        self._wheel_radius = 0.045 # in meters
        self._wheel_separation = 0.295 # in meters
        self._ticks_per_rev = 600 # Number of ticks per revolution of the motor

        # Current speeds
        self.left_speed = 0
        self.right_speed = 0
        self.angular_speed = 0

    def connect(self, connection_type=None):
        """
        Establish a connection to the robot using either serial or socket communication.
        Tries to connect using serial first, and falls back to socket if unsuccessful.
        """
        with self._internal_lock:
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
        try:
            self.serial_comm.connect()
            self.serial_running = self.serial_comm.is_connected()
            self.raw_data_queuedata_queue = self.serial_comm.raw_data_queue
        except Exception as e:
            print("Serial connection error:", e)

    def _connect_socket(self):
        """
        Attempt to establish a socket connection.
        """
        try:
            self.socket_comm.connect()
            self.socket_running = self.socket_comm.is_connected()
            self.raw_data_queue = self.socket_comm.raw_data_queue
        except Exception as e:
            print("Socket connection error:", e)

    def disconnect(self):
        """
        Disconnect both serial and socket connections.
        """
        with self._internal_lock:
            self.serial_comm.disconnect()
            self.socket_comm.disconnect()
            self.serial_running, self.socket_running = False, False

    def set_data_callback(self, callback):
        """
        Set a callback function to handle incoming data from the robot.
        """
        self.data_callback = callback


    def send_command(self, command):
        """
        Send a command to the robot using either serial or socket communication.
        """

        if self.serial_running:
            self.serial_comm.send_command(command)
        if self.socket_running:
            self.socket_comm.send_command(command)

    def set_speed(self, left_speed, right_speed):
        """
        Set the speed of the left and right wheels of the robot.
        """
        command_l = f"s l {left_speed}"
        command_r = f"s r {right_speed}"
        self.send_command(command_l)
        self.send_command(command_r)
        command_l = f"s l {left_speed}"
        command_r = f"s r {right_speed}"
        self.send_command(command_l)
        self.send_command(command_r)

    def move_forward(self, duration):
        """
        Move the robot forward for a specified duration.
        """
        self.set_speed(500, 500)  # Example values, adjust as needed
        self.set_speed(500, 500)  # Example values, adjust as needed
        time.sleep(duration)
        self.stop()

    def linear_ang_speed_to_freq(self, linear_speed, angular_speed):
        """
        Given desired linear and angular speed, calculate and send freq command to robot.
        """
        #Calculate right and left wheel linear velocity
        # left_speed = linear_speed - (angular_speed * self._wheel_separation / 2)
        # right_speed = linear_speed + (angular_speed * self._wheel_separation / 2)

        #Calculate right and left wheel angular velocity
        w_l = (linear_speed - (angular_speed * self._wheel_separation / 2)) / self._wheel_radius
        w_r = (linear_speed + (angular_speed * self._wheel_separation / 2)) / self._wheel_radius

        #Calculate right and left wheel tick frequency
        f_l = w_l / (2 * math.pi) * self._ticks_per_rev
        f_r = w_r / (2 * math.pi) * self._ticks_per_rev

        #Reduce the frequency to 3 decimal places
        f_l = round(f_l, 3)
        f_r = round(f_r, 3)

        self.set_speed(f_l, f_r)

    def stop(self):
        """
        Stop the robot by setting both wheel speeds to zero.
        """
        self.set_speed(0, 0)
        
    def request_sensor_data(self):
        """
        Request sensor data from the robot.
        """
        self.send_command("u 1 range")

    def get_status(self):
        """
        Request the current status of the robot.
        """
        self.send_command("GET_STATUS")



        


    def __del__(self):
        self.stop()
        self.disconnect()


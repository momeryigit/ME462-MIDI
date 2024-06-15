import threading
import time
from collections import deque
from .serial_comm import SerialCommunication
from .socket_comm import SocketCommunication


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
        self, port, baudrate=9600, timeout=1, ip="192.168.137.28", socket_port=8080
    ):
        if self._initialized:
            return

        self.port = port
        self.baudrate = baudrate
        self.ip = ip
        self.socket_port = socket_port
        self.timeout = timeout

        self.serial_comm = SerialCommunication(self.port, self.baudrate, self.timeout)
        self.socket_comm = SocketCommunication(self.ip, self.socket_port)
        self.serial_running = False
        self.socket_running = False
        self.data_queue = deque()
        self.data_callback = None

        self._internal_lock = threading.Lock()
        self._initialized = True

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
            self.data_queue = self.serial_comm.data_queue
        except Exception as e:
            print("Serial connection error:", e)

    def _connect_socket(self):
        """
        Attempt to establish a socket connection.
        """
        try:
            self.socket_comm.connect()
            self.socket_running = self.socket_comm.is_connected()
            self.data_queue = self.socket_comm.data_queue
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

    def send_command(self, command):
        """
        Send a command to the robot using either serial or socket communication.
        """
        with self._internal_lock:
            if self.serial_running:
                self.serial_comm.send_command(command)
            if self.socket_running:
                self.socket_comm.send_command(command)

    def set_data_callback(self, callback):
        """
        Set a callback function to handle incoming data from the robot.
        """
        with self._internal_lock:
            self.data_callback = callback
            self.serial_comm.set_data_callback(callback)
            self.socket_comm.set_data_callback(callback)

    def set_speed(self, left_speed, right_speed):
        """
        Set the speed of the left and right wheels of the robot.
        """
        command = f"s l {left_speed} r {right_speed}"
        self.send_command(command)

    def move_forward(self, duration):
        """
        Move the robot forward for a specified duration.
        """
        self.set_speed(400, 400)  # Example values, adjust as needed
        time.sleep(duration)
        self.stop()

    def turn(self, direction, steps, frequency):
        """
        Turn the robot in a specified direction for a given number of steps and frequency.
        """
        command = f"s {direction} {steps} {frequency}"
        self.send_command(command)

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
        self.disconnect()

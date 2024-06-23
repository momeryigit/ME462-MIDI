import threading
import serial
from .sensors import Sensors
import time

class SerialCommunication:
    """
    A singleton class to manage serial communication.
    Handles connection, disconnection, sending, and receiving data via a serial port.
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(SerialCommunication, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self, serial_port, baudrate=115200, timeout=1):
        if self._initialized:
            return
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.running = False
        self.polling_thread = None
        self._internal_lock = threading.Lock()
        self._initialized = True

        self.sensors = Sensors()  # Access singleton instance of Sensors class

    def connect(self):
        """
        Establish a serial connection to the specified port and baudrate.
        """
        with self._internal_lock:
            if not self.serial_connection or not self.serial_connection.is_open:
                self.serial_connection = serial.Serial(
                    self.serial_port, self.baudrate, timeout=self.timeout
                )
                self.running = True
                self.polling_thread = threading.Thread(target=self._poll_serial)
                self.polling_thread.start()

    def disconnect(self):
        """
        Close the serial connection and stop the polling thread.
        """
        with self._internal_lock:
            self.running = False
            if self.polling_thread:
                self.polling_thread.join()
            if self.serial_connection:
                self.serial_connection.close()

    def _poll_serial(self):
        """
        Poll the serial port for incoming data in a separate thread.
        """
        while self.running:
            if self.serial_connection.in_waiting > 0:
                with self._internal_lock:
                    raw_data = self.serial_connection.readline().decode("utf-8").strip()
                if raw_data:
                    self.sensors.parse_data(raw_data)
            time.sleep(0.01)
    

    def send_command(self, command):
        """
        Send a command through the serial connection.
        """
        if self.serial_connection:
            self.serial_connection.write((command + "\n").encode("utf-8"))

    def is_connected(self):
        """
        Check if the serial connection is currently open.
        """
        with self._internal_lock:
            return self.serial_connection.is_open if self.serial_connection else False

    def __del__(self):
        self.disconnect()

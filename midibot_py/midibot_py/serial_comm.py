import threading
import serial
import time
from collections import deque


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

    def __init__(self, port, baudrate=9600, timeout=1):
        if self._initialized:
            return
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.running = False
        self.data_callback = None
        self.polling_thread = None
        self.usonic_data = [12, 68, 94, 98]
        self._u_moving_avg = [[], [], [], []]
        self._internal_lock = threading.Lock()
        self._initialized = True

    def connect(self):
        """
        Establish a serial connection to the specified port and baudrate.
        """
        with self._internal_lock:
            if not self.serial_connection or not self.serial_connection.is_open:
                self.serial_connection = serial.Serial(
                    self.port, self.baudrate, timeout=self.timeout
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
                data = self.serial_connection.readline().decode("utf-8").strip()
                if data:
                    self._parse_serial_data(data)
            time.sleep(0.1)

    def send_command(self, command):
        """
        Send a command through the serial connection.
        """
        with self._internal_lock:
            if self.serial_connection:
                self.serial_connection.write((command + "\n").encode("utf-8"))

    def set_data_callback(self, callback):
        """
        Set a callback function to handle received data.
        """
        with self._internal_lock:
            self.data_callback = callback

    def is_connected(self):
        """
        Check if the serial connection is currently open.
        """
        with self._internal_lock:
            return self.serial_connection.is_open if self.serial_connection else False

    def _parse_serial_data(self, data):
        parts = data.split()
        if len(parts) < 2:
            return
        command = parts[0]
        if command == "u":  # Assuming 'u' is for sensor data updates
            sensor_id = parts[1]
            sensor_value = parts[2]        
            if self.data_callback:
                self._mov_avg(sensor_id, sensor_value)
                if sensor_id == "4":
                    self.data_callback(self.usonic_data)
            
    def _mov_avg(self, sensor_id, sensor_value):
        """
        Filter and process sensor data as needed.
        """
        index = int(sensor_id) - 1
        if len(self._u_moving_avg[index]) < 10:
            self._u_moving_avg[index].append(float(sensor_value))
            self.usonic_data[index] = (sum(self._u_moving_avg)) / len(self._u_moving_avg)
        else:
            self._u_moving_avg[index].append(float(sensor_value))
            self.usonic_data[index] = self.usonic_data[index] + (float(sensor_value) - self._u_moving_avg.popleft()) / 10
        

    def __del__(self):
        self.disconnect()
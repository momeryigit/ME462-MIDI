import threading
import serial
from .sensors import Sensors
import queue
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

    def __init__(self, serial_port, baudrate=9600, timeout=1):
        if self._initialized:
            return
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.running = False
        self.polling_thread = None
        self.usonic_data = [0.0, 0.0, 0.0, 0.0]
        self._u_moving_avg = [deque(maxlen=3) for _ in range(4)]  # Initialize a deque for each sensor
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
                    print(raw_data)
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

    def _parse_serial_data(self):
        with self._internal_lock:
            if self.sensor_1:
                print(self.sensor_1)
                if self.sensor_1 == -0.017:
                    self.sensor_1 = 260.0
                self._mov_avg("1", self.sensor_1)

                self.data_callback(self.usonic_data[0])
                self.sensor_1 = None
                # parts = data.split()
            # if len(parts) < 2:
            #     return
            # command = parts[0]
            # if command == "u":  # Assuming 'u' is for sensor data updates
            #     sensor_id = parts[1]
            #     sensor_value = parts[2]
            #     if sensor_value == "-0.017":
            #         sensor_value = "260.0"
            #     if self.data_callback:
            #         #self._mov_avg(sensor_id, sensor_value)
            #         if sensor_id == "1":
            #             self.data_callback(sensor_id, sensor_value)
                
    def _mov_avg(self, sensor_id, sensor_value):
        """
        Filter and process sensor data as needed.
        """
        
        index = int(sensor_id) - 1
        sensor_value = sensor_value
        if len(self._u_moving_avg[index]) < 3:
            self._u_moving_avg[index].append(sensor_value)
            self.usonic_data[index] = round(sum(self._u_moving_avg[index]) / len(self._u_moving_avg[index]), 3)
        else:
            # Calculate the new average
            _old = self._u_moving_avg[index].popleft()
            self._u_moving_avg[index].append(sensor_value)
            self.usonic_data[index] = round(self.usonic_data[index] + (sensor_value - _old) / 3, 3)

    def __del__(self):
        self.disconnect()

import threading
import serial
from .sensors import Sensors
import time
import errno
import logging

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
        self.possible_ports = []
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.running = False
        self.polling_thread = None
        self._internal_lock = threading.Lock()
        self._initialized = True
        self.data_callback = None

        self.sensors = Sensors()  # Access singleton instance of Sensors class
        self.mpu_timer = time.time()

    def connect(self):
        """
        Establish a serial connection to the specified port and baudrate.
        """
        try:
            with self._internal_lock:
                if not self.serial_connection or not self.serial_connection.is_open:
                    self.serial_connection = serial.Serial(
                        self.serial_port, self.baudrate, timeout=self.timeout
                    )
                    self.running = True
                    if self.polling_thread and self.polling_thread.is_alive():
                        return
                    self.polling_thread = threading.Thread(target=self._poll_serial)
                    self.polling_thread.start()
        except Exception as e:
            print("Serial connection error: ", e)
                
    def detect_ports(self):
        """
        Detect available serial ports.
        """
        for port in serial.tools.list_ports.comports():
            self.possible_ports.append(port.device)

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
            try:
                if self.serial_connection.in_waiting > 0:
                    with self._internal_lock:
                        raw_data = self.serial_connection.readline().decode("utf-8").strip()
                    if raw_data:
                        if self.data_callback:
                            self.data_callback(raw_data)
                        self.sensors.parse_data(raw_data)
                        if self.sensors.heartbeat:
                            self.send_command("h")
                            self.sensors.heartbeat = False
            except IOError as e:
                if e.errno == errno.EIO:
                    logging.error("Serial connection lost. Attempting to reconnect...")
                    if self.serial_connection:
                        try:
                            self.serial_connection.close()
                        except Exception as e:
                            logging.error(f"Error closing serial connection: {e}")
                        self.serial_connection = None

                    for attempt in range(1, 4):  # Attempt to reconnect 3 times
                        try:
                            self.connect()
                            if self.serial_connection:
                                logging.info("Reconnected to serial port successfully.")
                                break
                        except Exception as e:
                            logging.error(f"Reconnection attempt {attempt} failed: {e}")
                        
                        time.sleep(2 ** attempt)  # Exponential backoff strategy
                    else:
                        logging.critical("Failed to reconnect to serial port after multiple attempts.")
                        self.polling_thread.join()
                        self.__del__()  # Consider a more graceful shutdown approach
            
            if time.time() - self.mpu_timer > self.sensors.mpu_poll_interval:
                self.sensors.get_mpu_data()
                self.mpu_timer = time.time()
                
    
    def set_data_callback(self, callback):
        """
        Set a callback function to handle incoming data.
        """
        self.data_callback = callback

    def send_command(self, command):
        """
        Send a command through the serial connection.
        """
        with self._internal_lock:
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

import threading
import socket
from .sensors import Sensors


class SocketCommunication:
    """
    A singleton class to manage socket communication.
    Handles connection, disconnection, sending, and receiving data via a socket.
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(SocketCommunication, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self, ip="192.168.137.28", socket_port=8080):
        if self._initialized:
            return
        self.ip = ip
        self.socket_port = socket_port
        self.connection = None
        self.running = False
        self.polling_thread = None
        self._internal_lock = threading.Lock()
        self._initialized = True

        self.sensors = Sensors()  # Access singleton instance of Sensors class

    def connect(self):
        """
        Establish a socket connection to the specified IP and port.
        """
        with self._internal_lock:
            if not self.connection:
                self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.connection.settimeout(3)
                try:
                    self.connection.connect((self.ip, self.port))
                    self.running = True
                    self.polling_thread = threading.Thread(target=self._poll_socket)
                    self.polling_thread.start()
                except socket.timeout:
                    print("Socket connection timed out")
                except Exception as e:
                    print("Socket connection error:", e)

    def disconnect(self):
        """
        Close the socket connection and stop the polling thread.
        """
        with self._internal_lock:
            self.running = False
            if self.polling_thread:
                self.polling_thread.join()
            if self.connection:
                self.connection.close()
                self.connection = None

    def _poll_socket(self):
        """
        Poll the socket for incoming data in a separate thread.
        """
        while self.running:
            try:
                raw_data = self.receive_newline()
                if raw_data:
                    self.sensors.parse_data(raw_data)
            except Exception as e:
                print("Socket receive error:", e)

    def send_command(self, data):
        """
        Send a command through the socket connection.
        """
        with self._internal_lock:
            if self.connection:
                try:
                    data += "\n"
                    self.connection.send(data.encode())
                except Exception as e:
                    print("Socket send error:", e)

    def receive_newline(self):
        """
        Receive data from the socket until a newline character is encountered.
        """
        data = ""
        while self.running:
            try:
                chunk = self.connection.recv(1).decode("utf-8")
                if chunk == "\n":
                    return data
                data += chunk
            except Exception as e:
                print("Socket receive error:", e)

    def is_connected(self):
        """
        Check if the socket is currently connected.
        """
        with self._internal_lock:
            return self.connection is not None

    def __del__(self):
        self.disconnect()

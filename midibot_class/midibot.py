import threading

class DifferentialDriveRobot:
    _instance = None  # Class-level attribute to hold the single instance
    _lock = threading.Lock()  # Class-level lock to ensure thread safety

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # First check (without lock)
            with cls._lock:  # Acquire lock to enter critical section
                if cls._instance is None:  # Double-check (with lock)
                    cls._instance = super(DifferentialDriveRobot, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self, port, baudrate=9600, timeout=1):
        if self._initialized:
            return
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.polling_thread = None
        self.running = False
        self.data_callback = None
        self._internal_lock = threading.Lock()
        self._initialized = True

    def connect(self):
        with self._internal_lock:
            if not self.serial_connection or not self.serial_connection.is_open:
                self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self.running = True
                self.polling_thread = threading.Thread(target=self._poll_serial)
                self.polling_thread.start()

    def disconnect(self):
        with self._internal_lock:
            self.running = False
            if self.polling_thread:
                self.polling_thread.join()
            if self.serial_connection:
                self.serial_connection.close()

    def _poll_serial(self):
        while self.running:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8').strip()
                self._parse_serial_data(data)
            time.sleep(0.1)

    def send_command(self, command):
        with self._internal_lock:
            if self.serial_connection:
                self.serial_connection.write((command + '\n').encode('utf-8'))

    def set_data_callback(self, callback):
        with self._internal_lock:
            self.data_callback = callback

    def set_speed(self, left_speed, right_speed):
        command = f's l {left_speed} r {right_speed}'
        self.send_command(command)

    def stop(self):
        self.set_speed(0, 0)

    def move_forward(self, duration):
        self.set_speed(400, 400)  # Example values, adjust as needed
        time.sleep(duration)
        self.stop()

    def turn(self, direction, steps, frequency):
        command = f's {direction} {steps} {frequency}'
        self.send_command(command)

    def request_sensor_data(self):
        self.send_command('u 1 range')

    def is_connected(self):
        with self._internal_lock:
            return self.serial_connection.is_open if self.serial_connection else False

    def get_status(self):
        self.send_command('GET_STATUS')

    def _parse_serial_data(self, data):
        parts = data.split()
        if len(parts) < 2:
            return
        command = parts[0]
        if command == 'u':  # Assuming 'u' is for sensor data updates
            sensor_id = parts[1]
            sensor_value = parts[2]
            with self._internal_lock:
                if self.data_callback:
                    self.data_callback(sensor_id, sensor_value)
        elif command == 's':  # Assuming 's' is for stepper motor control feedback
            direction = parts[1]
            steps = parts[2]
            frequency = parts[3]
            # Handle motor control feedback if needed
        # Add more command parsing as needed

    def __del__(self):
        self.disconnect()

from collections import deque

class Sensors:
    """
    A class to manage all received data from sensors. Singleton class.
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        """
        Ensure that only one instance of the class is created (singleton pattern).
        """
        if cls._instance is None:
            cls._instance = super(Sensors, cls).__new__(cls)
            cls._instance._initialize(*args, **kwargs)
        return cls._instance

    def _initialize(self, u_count=4, b_count=4, imu_connected=False, u_moving_avg_len=3):
        """
        Initialize the sensor data storage and moving average configuration.

        Args:
            u_count (int): Number of ultrasonic sensors.
            b_count (int): Number of bumper switches.
            imu_connected (bool): Flag indicating if IMU is connected.
            u_moving_avg_len (int): Length of the moving average window for ultrasonic sensors.
        """
        if getattr(self, '_initialized', False):
            return

        self.u_moving_avg_len = u_moving_avg_len
        self.u_sonic_data = {f'u_{i + 1}': 0.0 for i in range(u_count)}  # Ultrasonic sensor data
        self.u_moving_avg = [deque(maxlen=self.u_moving_avg_len) for _ in range(u_count)]  # Deques for moving average

        if imu_connected:
            self.imu_data = {'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0, 'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0}  # IMU sensor data

        self.b_switch_data = {f'b_{i + 1}': False for i in range(b_count)}  # Bumper switch data
        
        self._initialized = True

    def request_sensor_data(self, sensor_type):
        """
        Request sensor data from the robot.

        Args:
            sensor_type (str): The type of sensor data to request.
        """
        if sensor_type == "u":
            return self.u_sonic_data
        elif sensor_type == "i":
            return self.imu_data
        elif sensor_type == "b":
            return self.b_switch_data


    def parse_data(self, data):
        """
        Parse the received data and update the sensor data dictionaries.
        
        Args:
            data (str): A string containing sensor data in the format "<type> <id> <values>".
        """
        parts = data.split()
        if len(parts) < 2:
            return
        
        identifier = parts[0]
        sensor_id = parts[1]
        sensor_data = parts[2:]

        if identifier == "u":
            self._manage_u_sonic_data(sensor_id, sensor_data)
        elif identifier == "i":
            self._manage_imu_data(sensor_data)
        elif identifier == "b":
            self._manage_b_switch_data(sensor_id, sensor_data[0])
        elif identifier == "s":
            pass
        else:
            print(f"Unknown identifier: {identifier}")

    def _moving_avg(self, u_id, u_value):
        """
        Calculate the moving average of the ultrasonic sensor data to smooth out the noise.
        
        Args:
            u_id (str): The ID of the ultrasonic sensor.
            u_value (float): The latest reading from the ultrasonic sensor.
        """
        index = int(u_id) - 1
        u_value = round(float(u_value), 3)
        if u_value == -0.017:
            u_value = 260.0
        elif u_value == -0.032:
            u_value = "sensor disconnected"
        self.u_moving_avg[index].append(u_value)
        
        # Calculate the moving average
        self.u_sonic_data[f'u_{u_id}'] = round(sum(self.u_moving_avg[index]) / len(self.u_moving_avg[index]), 3)

    def _manage_u_sonic_data(self, u_id, u_data):
        """
        Manage ultrasonic sensor data.
        
        Args:
            u_id (str): The ID of the ultrasonic sensor.
            u_data (list): The list containing the data from the ultrasonic sensor.
        """
        self._moving_avg(u_id, u_data[0])

    def _manage_imu_data(self, i_data):
        """
        Manage IMU sensor data.
        
        Args:
            i_data (list): The list containing the data from the IMU sensor.
        """
        self.imu_data['accel_x'] = float(i_data[0])
        self.imu_data['accel_y'] = float(i_data[1])
        self.imu_data['accel_z'] = float(i_data[2])
        self.imu_data['gyro_x'] = float(i_data[3])
        self.imu_data['gyro_y'] = float(i_data[4])
        self.imu_data['gyro_z'] = float(i_data[5])

    def _manage_b_switch_data(self, b_id, b_data):
        """
        Manage bumper switch data.
        
        Args:
            b_id (str): The ID of the bumper switch.
            b_data (str): The state of the bumper switch (True/False).
        """
        b_state = b_data.lower() == 'true'
        self.b_switch_data[f'b_{b_id}'] = b_state

# Example usage
# sensors1 = Sensors(u_count=4, b_count=4, imu_connected=True, u_moving_avg_len=3)
# sensors2 = Sensors(u_count=6, b_count=6, imu_connected=False, u_moving_avg_len=5)

# # Simulated sensor data
# sensors1.parse_data("u 1 230.24738")
# sensors1.parse_data("u 1 240.12345")
# sensors1.parse_data("u 1 250.56789")

# sensors1.parse_data("b 3 False")
# sensors1.parse_data("i 1 0.1 6.5 8.2 20.243 18.3 9.1")

# print(sensors1.u_sonic_data)  # Output will show the moving average
# print(sensors1.b_switch_data)
# print(sensors1.imu_data)

# print(sensors1 is sensors2)  # True, both are the same instance

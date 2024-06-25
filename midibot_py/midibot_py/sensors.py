from .MPU import MPU as UPS
from collections import deque


#from .serial_comm import SerialCommunication

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

    def _initialize(self, stepper_ids=[1,2], u_ids=[1,2,3,4], b_ids=[1,2,3,4], imu_connected=True, u_median_filter_len=3):
        """
        Initialize the sensor data storage, steppers, and median_filter for ultrasonics configuration.

        Args (as specified from config file):
            u_ids ([int]): Integer array of ids of ultrasonic sensors intialized.
            b_ids ([int]): Integer array of ids of bumper switches to be initialized.
            imu_connected (bool): Flag indicating if IMU is connected.
            u_median_filter_len (int): Length of the median filter window for ultrasonic sensors.
        """
        if getattr(self, '_initialized', False):
            return
        
        self.stepper_ids = stepper_ids
        self.u_ids = u_ids
        self.b_ids = b_ids
        self.imu_connected = imu_connected
        
        self.heartbeat = False # Flag to indicate if heartbeat is received
        self.handshake = False # Flag to indicate if handshake is successful
        
        self.ups_poll_interval = 1 # Polling interval for UPS sensor in seconds
        self.ups_discharging = False

        # UPS intitialization
        try:
            self.ups = UPS(addr=0x41)
        except Exception as e:
            self.ups = None
            print("UPS sensor not connected: ", e)
            
        if self.ups:
            self.ups_data = {'bus_voltage': 0.0, 'shunt_voltage': 0.0, 'current': 0.0, 'power': 0.0, 'percent': 0.0, 'discharging': False}
        else:
            self.ups_data = None
        
        
        self.u_median_filter_len = u_median_filter_len

        self.u_sonic_data = {f'u_{u_id}': 0.0 for u_id in u_ids}  # Ultrasonic sensor data
        self.u_median_filter = {f'u_{u_id}': deque(maxlen=u_median_filter_len) for u_id in u_ids}  # Deques for u_sonic data median filter
        
        # IMU sensor data
        if imu_connected:
            self.imu_data = {'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0, 'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0}

        self.b_switch_data = {f'b_{b_id}': False for b_id in b_ids}  # Bumper switch data

        # Stepper motor counts
        if stepper_ids:
            self.stepper_count = {}
            if 1 in stepper_ids:
                self.stepper_count['S_L'] = 0
            if 2 in stepper_ids:
                self.stepper_count['S_R'] = 0
                        
        self._initialized = True
        

    def request_sensor_data(self, sensor_type="u"):
        """
        Request sensor data from the robot.

        Args:
            sensor_type (str): The type of sensor data to request.
            "u" : ultrasonic data stored in self.u_sonic_data
            "i" : IMU data stored in self.imu_data
            "b" : bumper switch data stored in self.b_switch_data
            "sc" : stepper count data stored in self.stepper_count
        """
        if sensor_type == "u":
            return self.u_sonic_data
        elif sensor_type == "i":
            return self.imu_data
        elif sensor_type == "b":
            return self.b_switch_data
        elif sensor_type == "sc":
            return self.stepper_count
        else:
            return "Unknown request. Please specify a valid sensor type (u, i, b, sc)."

    def parse_data(self, data):
        """
        Parse the received data and update the sensor data dictionaries.
        
        Args:
            data (str): A string containing sensor data in the format "<type> <id> <values>".
            data format: "<identifier> <id> <values>"
            <values> could be multiple data.
        """
        parts = data.split()
        nonsensor = False
        try:
            identifier = parts[0]
            if identifier == "h":
                self._manage_heartbeat_data()
                return
            elif identifier == "handshake":
                self.handshake = True
                return
            elif identifier not in ["u", "i", "b", "sc"]:
                print("PICO message:", (" ").join(parts))
                return
            
            sensor_id = parts[1]
            sensor_data = parts[2:]
        except IndexError:
            nonsensor = True
            

        if identifier == "u":
            self._manage_u_sonic_data(sensor_id, sensor_data[0])
        elif identifier == "i":
            self._manage_imu_data(sensor_data)
        elif identifier == "b":
            self._manage_b_switch_data(sensor_id, sensor_data[0])
        elif identifier == "sc":
            self._manage_stepper_count(sensor_id, sensor_data)
        elif nonsensor:
            print(data)
        else:
            print("Unknown identifier: ")

    def _median_filter(self, u_id, u_value):
        """
        Implement a median filter for ultrasonic sensor data.
        
        Args:
            u_id (str): The ID of the ultrasonic sensor.
            u_value (float): The latest reading from the ultrasonic sensor.
        """
        u_value = round(float(u_value), 3)
        if u_value == -0.017:
            u_value = 260.0
        elif u_value == -0.032:
            self.u_sonic_data[f'u_{u_id}'] = "sensor disconnected"
            return

        self.u_median_filter[f'u_{u_id}'].append(u_value)

        if len(self.u_median_filter[f'u_{u_id}']) == self.u_median_filter_len:
            sorted_values = sorted(self.u_median_filter[f'u_{u_id}'])
            median_value = sorted_values[self.u_median_filter_len // 2]
            self.u_sonic_data[f'u_{u_id}'] = median_value
            # Reset the deque
            self.u_median_filter[f'u_{u_id}'].clear()
    def _manage_u_sonic_data(self, u_id, u_data):
        """
        Manage ultrasonic sensor data.
        
        Args:
            u_id (str): The ID of the ultrasonic sensor.
            u_data (list): The list containing the data from the ultrasonic sensor.
        """
        self._median_filter(u_id, u_data)

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
    
    def _manage_stepper_count(self, stepper_id, count_data):
        """
        Manage stepper motor count data.
        
        Args:
            stepper_id (str): The ID of the stepper motor.
            count_data (list): The list containing the count data from the stepper motor.
        """
        count = int(count_data[0])
        if stepper_id == '1':
            self.stepper_count['S_L'] = count
        elif stepper_id == '2':
            self.stepper_count['S_R'] = count
    
    def _manage_heartbeat_data(self):
        """
        Manage heartbeat data.
        """
        print("Heartbeat received")
        self.heartbeat = True
        pass

    def get_ups_data(self):
        """
        Get the UPS sensor data.
        
        Return a dictionary of the UPS data at time of request and updates the data.
        """
        if not self.ups:
            return None
        bus_voltage = self.ups.getBusVoltage_V()             # voltage on V- (load side)
        shunt_voltage = self.ups.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
        current = self.ups.getCurrent_mA()                   # current in mA
        power = self.ups.getPower_W()                        # power in W
        p = (bus_voltage - 9)/3.6*100
        if(p > 100):p = 100
        if(p < 0):p = 0
        
        if current < 0:
            self.ups_discharging = True
        else:
            self.ups_discharging = False
        
        self.ups_data['bus_voltage'] = bus_voltage
        self.ups_data['shunt_voltage'] = shunt_voltage
        self.ups_data['current'] = current
        self.ups_data['power'] = power
        self.ups_data['percent'] = p
        self.ups_data['discharging'] = self.ups_discharging
        
        return self.ups_data

# Example usage
# sensors1 = Sensors(u_count=4, b_count=4, imu_connected=True, u_median_filter_len=3)
# sensors2 = Sensors(u_count=6, b_count=6, imu_connected=False, u_median_filter_len=5)

# # Simulated sensor data
# sensors1.parse_data("u 1 230.24738")
# sensors1.parse_data("u 1 240.12345")
# sensors1.parse_data("u 1 250.56789")

# sensors1.parse_data("b 3 False")
# sensors1.parse_data("i 1 0.1 6.5 8.2 20.243 18.3 9.1")

# print(sensors1.b_switch_data)
# print(sensors1.imu_data)

# print(sensors1 is sensors2)  # True, both are the same instance

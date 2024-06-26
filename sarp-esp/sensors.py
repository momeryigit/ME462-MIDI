from machine import Pin, Timer, I2C
from hcsr04 import HCSR04
from imu import MPU6050

class Sensors:
    """
    A class to manage different types of sensors including ultrasonic sensors, IMU sensors, and bumper switches.
    """

    def __init__(self):
        """
        Initialize the Sensors class with default configurations and empty sensor dictionaries.
        """
        self.types = {
            "ultrasonic": {"poll_rate": 5, "sensor": {}},
            "imu": {"poll_rate": 10, "sensor": {}},
            "bumper": {"poll_rate": 1, "sensor": {}},
        }
        self.timers = {}
        self.IMU_flag = False
        self.ultrasonic_flag = False
        self.default_emergency_behavior = False

    def create_ultrasonic(self, id, trigger, echo):
        """
        Create and initialize an ultrasonic sensor.

        Parameters:
        id : str
            Identifier for the sensor.
        trigger : int
            GPIO pin number for the trigger.
        echo : int
            GPIO pin number for the echo.
        """
        self.types["ultrasonic"]["sensor"][id] = HCSR04(trigger, echo)

    def create_imu(self, id, sda, scl):
        """
        Create and initialize an IMU sensor.

        Parameters:
        id : str
            Identifier for the sensor.
        sda : int
            GPIO pin number for SDA.
        scl : int
            GPIO pin number for SCL.
        """
        try:
            i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)
            self.types["imu"]["sensor"][id] = MPU6050(i2c)
        except Exception as e:
            print("Error initializing IMU sensor:", e)

    def create_bumper(self, id, pin):
        """
        Create and initialize a bumper switch.

        Parameters:
        id : str
            Identifier for the bumper switch.
        pin : int
            GPIO pin number for the bumper switch.
        """
        self.types["bumper"]["sensor"][id] = Pin(pin, Pin.IN, Pin.PULL_DOWN)

    def poll_ultrasonic(self, id):
        """
        Poll an ultrasonic sensor and print its distance measurement.

        Parameters:
        id : str
            Identifier for the ultrasonic sensor.
        """
        print("u " + str(id) + " " + str(self.types["ultrasonic"]["sensor"][id].distance_cm()))

    def poll_imu(self, id):
        """
        Poll an IMU sensor and print its accelerometer and gyroscope data.

        Parameters:
        id : str
            Identifier for the IMU sensor.
        """
        imu = self.types["imu"]["sensor"][id]
        print(f"i {id} {imu.accel.x} {imu.accel.y} {imu.accel.z} {imu.gyro.x} {imu.gyro.y} {imu.gyro.z}")

    def poll_bumper(self, id):
        """
        Poll a bumper switch and print its state.

        Parameters:
        id : str
            Identifier for the bumper switch.
        """
        print("b " + str(id) + " " + str(self.types["bumper"]["sensor"][id].value()))
    
    def set_u_sonic_flag(self):
        """
        Set the ultrasonic sensor polling flag to True.
        """
        self.ultrasonic_flag = True

    def set_imu_flag(self):
        """
        Set the IMU sensor polling flag to True.
        """
        self.IMU_flag = True

    def start_polling_timers(self):
        """
        Start polling timers for all initialized sensors. If a timer is already running, it resets the timer.
        """
        # Start polling for ultrasonic sensors
        if self.types["ultrasonic"]["sensor"]:
            if "ultrasonic" in self.timers:
                self.timers["ultrasonic"].deinit()
            self.timers["ultrasonic"] = Timer()
            self.timers["ultrasonic"].init(
                freq=self.types["ultrasonic"]["poll_rate"], mode=Timer.PERIODIC, callback=lambda t: self.set_u_sonic_flag()
            )

        # Start polling for IMU sensors
        if self.types["imu"]["sensor"]:
            if "imu" in self.timers:
                self.timers["imu"].deinit()
            self.timers["imu"] = Timer()
            self.timers["imu"].init(
                freq=self.types["imu"]["poll_rate"], mode=Timer.PERIODIC, callback=lambda t: self.set_imu_flag()
            )

    def set_poll_rate(self, sensor_type, poll_rate):
        """
        Set the polling rate for a specific type of sensor.

        Parameters:
        sensor_type : str
            The type of sensor (e.g., "ultrasonic", "imu", "bumper").
        poll_rate : int
            The polling rate in Hz.
        """
        self.types[sensor_type]["poll_rate"] = poll_rate

    def stop_polling(self, sensor_type):
        """
        Stop the polling timer for a specific type of sensor.

        Parameters:
        sensor_type : str
            The type of sensor (e.g., "ultrasonic", "imu", "bumper").
        """
        self.timers[sensor_type].deinit()
        del self.timers[sensor_type]
    
    def send_sensory_data(self):
        """
        Send sensory data by polling sensors if their respective flags are set.
        """
        if self.ultrasonic_flag:
            # Poll all ultrasonic sensors
            for id in self.types["ultrasonic"]["sensor"]:
                self.poll_ultrasonic(id)
            self.ultrasonic_flag = False
        
        if self.IMU_flag:
            # Poll all IMU sensors
            for id in self.types["imu"]["sensor"]:
                self.poll_imu(id)
            self.IMU_flag = False

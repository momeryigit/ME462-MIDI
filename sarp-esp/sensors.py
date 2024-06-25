from machine import Pin, Timer, I2C
from hcsr04 import HCSR04
from imu import MPU6050

class Sensors:
    def __init__(self):
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
        self.types["ultrasonic"]["sensor"][id] = HCSR04(trigger, echo)

    def create_imu(self, id, sda, scl):
        try:
            i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)
            self.types["imu"]["sensor"][id] = MPU6050(i2c)
        except Exception as e:
            print("Error initializing imu sensor:", e)

    def create_bumper(self, id, pin):
        self.types["bumper"]["sensor"][id] = Pin(pin, Pin.IN, Pin.PULL_DOWN)

    def poll_ultrasonic(self, id):
        print(("u " + str(id) + " " + str(self.types["ultrasonic"]["sensor"][id].distance_cm())))

    def poll_imu(self, id):
        imu = self.types["imu"]["sensor"][id]
        print(
            f"i {id} {imu.accel.x} {imu.accel.y} {imu.accel.z} {imu.gyro.x} {imu.gyro.y} {imu.gyro.z}"
        )

    def poll_bumper(self, id):
        print("b " + str(id) + " " + str(self.types["bumper"]["sensor"][id].value()))

    def start_polling(self, sensor_type, id):
        if sensor_type in self.timers:
            self.timers[sensor_type].deinit()  # stop previous timer
        self.timers[sensor_type] = Timer(-1)
        self.timers[sensor_type].init(
            freq=self.types[sensor_type]["poll_rate"],
            mode=Timer.PERIODIC,
            callback=lambda t: getattr(self, f"poll_{sensor_type}")(id),
        )

    def set_poll_rate(self, sensor_type, poll_rate):
        self.types[sensor_type]["poll_rate"] = poll_rate

    def stop_polling(self, sensor_type):
        self.timers[sensor_type].deinit()
        del self.timers[sensor_type]
    def send_sensory_data(self):
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
    

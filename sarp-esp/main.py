import time
import builtins
from machine import Pin, PWM, Timer, WDT
#from stepper import Stepper
from serial import SerialComm
from hcsr04 import HCSR04
from imu import MPU6050
from machine import Pin, I2C, freq
from ADS1115 import *
import neopixel
import _thread
import sys
import gc
import os


#i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
#imu = MPU6050(i2c)
# #BUMPER1 = Pin(13, Pin.IN, Pin.PULL_DOWN)
# #ADS1115_ADDRESS = 0x48
# #adc = ADS1115(ADS1115_ADDRESS, i2c=i2c)
# #adc.setVoltageRange_mV(ADS1115_RANGE_6144)
# #adc.setConvRate(ADS1115_128_SPS)
# #adc.setMeasureMode(ADS1115_CONTINUOUS)
#

class Stepper:
    # Constructor for the Stepper class
    def __init__(self, step_pin, dir_pin, count_pin, led_pin, invert_dir=False):
        # Initialize pins and variables
        self.step_pin = PWM(Pin(step_pin))  # Step pin for stepper motor
        self.dir_pin = Pin(dir_pin, Pin.OUT)  # Direction pin for stepper motor
        self.count_pin = Pin(count_pin, Pin.IN, Pin.PULL_DOWN)  # Count pin for stepper motor

        self.decelerate = False  # Flag to indicate if the motor is decelerating
        if invert_dir == True:
            self.dir = 1
        else:
            self.dir = 0
        self.min_freq = 15  # Minimum frequency for stepper motor
        self.max_freq = 4000  # Maximum frequency for stepper motor
        self.np = np = neopixel.NeoPixel(Pin(led_pin), 10)  # NeoPixel for LED control
        self.led_index = -1  # Index for LED control
        
        self.freq = 0  # Current frequency of stepper motor
        self.pos = 0  # Current position of stepper motor
        #self.count_pin.irq(trigger=Pin.IRQ_RISING, handler=self._step_callback)  # Interrupt for step callback
        self.direction = 0  # Represents active direction of stepper


        self.step_size = 0  # Step size for frequency change
        self.target_freq = 0  # Target frequency for stepper motor
        self.timer = Timer()  # Timer for frequency change

    # Method to accelerate the stepper motor to a target frequency
    def accelerate(self, target_freq): 
        if target_freq < self.freq:
            self.decelerate = True
        else:
            self.decelerate = False
        freq_diff = abs(target_freq - self.freq)
        self.target_freq = target_freq
        self.step_size = 50  # Hz
        timer_period = 20  # ms
        # Start the timer
        self.timer.init(period=timer_period, mode=Timer.PERIODIC, callback=self._change_freq)

    # Callback method to change the frequency of the stepper motor
    def _change_freq(self, timer):
        if self.freq < self.target_freq and self.decelerate == False:
            self.freq += self.step_size
            self.step_pin.freq(abs(self.freq))
        elif self.freq > self.target_freq and self.decelerate == True:
            self.freq -= self.step_size
            self.step_pin.freq(abs(self.freq))
        else:
            # Stop the timer when the target frequency is reached
            self.freq = self.target_freq
            self.step_pin.freq(abs(self.freq))
            self.timer.deinit()

    # Method to step the stepper motor at a certain frequency
    def step(self, freq):
        self.freq = freq  
        self.step_pin.freq(abs(self.freq))
        self.step_pin.duty_u16(int((30/100)*65_535))  

    # Method to stop the stepper motor
    def stop(self):
        self.freq = 0
        self.step_pin.duty_u16(0)
    
    # Callback method for step interrupt
    def _step_callback(self, pin):
        if self.dir_pin.value() == 1:
            self.pos += 1
        else:
            self.pos -= 1
        if self.pos >= 1200 or self.pos <= -1200:
            self.pos = 0

    def update_leds(self, led_index):
        # Update the LEDs
        for i in range(10):
            if i <= led_index:
                self.np[i] = (0, 30, 60)
            else:
                self.np[i] = (0, 0, 0)
        self.np.write()

    # Property for frequency
    @property
    def freq(self):
        return self._freq
    
    # Setter for frequency
    @freq.setter
    def freq(self, value):
        if abs(value) > self.max_freq:
            self._freq = self.max_freq
        else:
            self._freq = value

        if abs(self._freq) < self.min_freq and self.decelerate == True:
            self._freq = -1 * self.min_freq
        elif abs(self._freq) < self.min_freq and self.decelerate == False:
            self._freq = self.min_freq

        
        if self._freq<0:
            self.dir_pin.value(1-self.dir)
            self.direction = -1
        else:
            self.dir_pin.value(self.dir)
            self.direction = 1
            
        steps = (self.max_freq - self.min_freq) / 10

        if self._freq == 0 or ((abs(value) - self.min_freq) / steps) <= 0:
            self.update_leds(-1)  # Turn off all LEDs
            return
        
        led_index = round((abs(self._freq) - self.min_freq) / steps)
        if self.led_index != led_index:
            self.led_index = led_index
            self.update_leds(self.led_index)

        
        
        
s_r = Stepper(2, 3, 7, 28)
s_l = Stepper(4, 5, 8, 28, invert_dir = True) #step_pin,dir_pin, steps_per_rev=600,invert_dir=False
en_pin = Pin(6, Pin.OUT)

class Sensors():
    def __init__(self):
        self.types = {
            'ultrasonic': {'poll_rate': 50, 'sensor': {}}, 
            'imu': {'poll_rate': 10, 'sensor': {}},
            'bumper': {'poll_rate': 1, 'sensor': {}},
            'adc': {'poll_rate': 1, 'sensor': {}},
        }
        self.timers = {}

    def create_ultrasonic(self, id, trigger, echo):
        self.types['ultrasonic']['sensor'][id] = HCSR04(trigger, echo)

    def create_imu(self, id, sda, scl):
        i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)
        self.types['imu']['sensor'][id] = MPU6050(i2c)
    
    def create_bumper(self, id, pin):
        self.types['bumper']['sensor'][id] = Pin(pin, Pin.IN, Pin.PULL_DOWN)


    def poll_ultrasonic(self, id):
        print('u '+ str(id) + " " + str(self.types['ultrasonic']['sensor'][id].distance_cm())) 
       
    
    def poll_imu(self, id):
        print('i '+ str(self.types['imu']['sensor'][id]) + " " + self.types['imu']['sensor'][id].accel.x + " " + self.types['imu']['sensor'][id].accel.y + " " + self.types['imu']['sensor'][id].accel.z + " " + self.types['imu']['sensor'][id].gyro.x + " " + self.types['imu']['sensor'][id].gyro.y + " " + self.types['imu']['sensor'][id].gyro.z)

    def poll_bumper(self, id):
        print('b '+ str(self.types['bumper']['sensor'][id]) + " " + self.types['bumper']['sensor'][id].value())

    def start_polling(self, sensor_type, id):
        if sensor_type in self.timers:
            self.timers[sensor_type].deinit()  # stop previous timer
        self.timers[sensor_type] = Timer(-1)
        self.timers[sensor_type].init(freq=self.types[sensor_type]['poll_rate'], mode=Timer.PERIODIC, callback=lambda t: getattr(self, f'poll_{sensor_type}')(id))

    def set_poll_rate(self, sensor_type, poll_rate):
        self.types[sensor_type]['poll_rate'] = poll_rate

    def stop_polling(self, sensor_type):
        self.timers[sensor_type].deinit()
        del self.timers[sensor_type]

    
# def BUMPERprint():
#     # print BUMPER readings
#     print("Bumper1:",BUMPER1.value())
#     
# def ADCprint():
#     #print ADC readings
#     voltage0 = readChannel(ADS1115_COMP_0_GND)
#     voltage1 = readChannel(ADS1115_COMP_1_GND)
#     voltage2 = readChannel(ADS1115_COMP_2_GND)
#     voltage3 = readChannel(ADS1115_COMP_3_GND)
#     print("ADC1: {:<8.2f} ADC2: {:<8.2f} ADC3: {:<8.2f} ADC4: {:<8.2f}".format(voltage0, voltage1, voltage2, voltage3))
#     #print ADC readings
#     
# def readChannel(channel):
#     adc.setCompareChannels(channel)
#     voltage = adc.getResult_V()
#     return voltage

# def heartbeat(comms, write_lock):
#     global beats
#     beats += 1
#     with write_lock:
#         comms.send_message('h ' + str(beats))


Sensors = Sensors()
Sensors.create_ultrasonic(1, 9, 10)
Sensors.create_ultrasonic(2, 11, 12)
Sensors.create_ultrasonic(3, 13, 14)
Sensors.create_ultrasonic(4, 15, 16)

ultrasonic_flag = False
IMU_flag = False


def IMUprint():
    global IMU_flag
    IMU_flag = True
    
def getdist():
    global ultrasonic_flag
    ultrasonic_flag = True
    
def send_sensory_data(comms):
    global ultrasonic_flag, IMU_flag
    if ultrasonic_flag == True:
        Sensors.poll_ultrasonic(1)
        Sensors.poll_ultrasonic(2)
        Sensors.poll_ultrasonic(3)
        Sensors.poll_ultrasonic(4)
        ultrasonic_flag = False
    # if IMU_flag == True:
    #     ax=round(imu.accel.x,2)
    #     ay=round(imu.accel.y,2)
    #     az=round(imu.accel.z,2)
    #     gx=round(imu.gyro.x,2)
    #     gy=round(imu.gyro.y,2)
    #     gz=round(imu.gyro.z,2)
    #     comms.send_message("i "+ "1 " + str(ax) + " " + str(ay) + " " + str(az) + " "+ str(gx) + " "+ str(gy) + " "+ str(gz))
    #     IMU_flag = False


# timer_for_heartbeat =Timer()
timer_for_ultrasonic = Timer()
#timer_for_IMU = Timer()
# timer_for_ADC = Timer()
# timer_for_BUMPER = Timer()
# print(gc.mem_free())
# 
# main_running = True
# 
def main():
    time.sleep(3)
    freq(120000000) # set the CPU frequency to 240 MHz
    print(freq())
    comms = None
    for i in range(3):
        try:
            comms = SerialComm()
            break
        except Exception as e:
            print(f'Failed to initialize serial port. {str(e)}')
            pass


    timer_for_ultrasonic.init(freq=0.5, mode=Timer.PERIODIC, callback =lambda t: getdist())     
 #   timer_for_IMU.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: IMUprint())
#     #timer_for_heartbeat.init(freq=1, mode=Timer.PERIODIC, callback=lambda t:heartbeat(comms,write_lock))
#   #  timer_for_ADC.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: ADCprint())
#    # timer_for_BUMPER.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: BUMPERprint())

    en_pin.value(1)

    while True:
        try:
            msg = comms.read_parse()
        except Exception as e:
            print(type(e))
        send_sensory_data(comms)
        if msg != None:
            mystring = ""
            for data in msg:
                mystring += str(data) + " "
            print(mystring)
            if msg[0] == 's':
                if msg[1] == 'r':
                    s1 = s_r
                elif msg[1] == 'l':
                    s1 = s_l
                else:
                    continue
                if int(float(msg[2])) == 0 : #To avoid low frequency error.
                    s1.stop()
                    print(s1.pos)
                    en_pin.value(1)
                    if s_r.freq == 0 and s_l.freq == 0:
                        pass
                    continue
                en_pin.value(0)
                s1.step(s1.freq)
                s1.accelerate(int(float(msg[2])))

        gc.collect()
    
    
if __name__ == '__main__':
    main()
    











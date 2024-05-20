import time
import builtins
from machine import Pin, PWM, Timer
from stepper import Stepper
from serial import SerialComm
from socketclass import WifiServer
from hcsr04 import HCSR04
from imu import MPU6050
from machine import Pin, I2C
from ADS1115 import *

beats = 0

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)
BUMPER1 = Pin(13, Pin.IN, Pin.PULL_DOWN)

ADS1115_ADDRESS = 0x48
adc = ADS1115(ADS1115_ADDRESS, i2c=i2c)
adc.setVoltageRange_mV(ADS1115_RANGE_6144)
adc.setConvRate(ADS1115_128_SPS)
adc.setMeasureMode(ADS1115_CONTINUOUS)

def IMUprint():
    # print IMU readings
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2),
    print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ")
    # print IMU readings
    
def BUMPERprint():
    # print BUMPER readings
    print("Bumper1:",BUMPER1.value())
    
def ADCprint():
    #print ADC readings
    voltage0 = readChannel(ADS1115_COMP_0_GND)
    voltage1 = readChannel(ADS1115_COMP_1_GND)
    voltage2 = readChannel(ADS1115_COMP_2_GND)
    voltage3 = readChannel(ADS1115_COMP_3_GND)
    print("ADC1: {:<8.2f} ADC2: {:<8.2f} ADC3: {:<8.2f} ADC4: {:<8.2f}".format(voltage0, voltage1, voltage2, voltage3))
    #print ADC readings
    
def readChannel(channel):
    adc.setCompareChannels(channel)
    voltage = adc.getResult_V()
    return voltage

def heartbeat(comms):
    global beats
    beats += 1
    comms.send_message('h ' + str(beats))

distance = 0
def getdist(ultrasonic, comms):
    global distance
    distance = ultrasonic.distance_cm()
    comms.send_message(distance)
def toggle(mypin):
    mypin.value(not mypin.value())


def main():
    s_r = Stepper(2, 3, steps_per_rev=600, speed_sps=10, invert_dir=True)
    s_l = Stepper(4, 5, steps_per_rev=600, speed_sps=10, timer_id=3) #step_pin,dir_pin,en_pin=None,steps_per_rev=600,speed_sps=10,invert_dir=False,timer_id=2
    ultrasonic = HCSR04(14, 15)
    en_pin = Pin(6, Pin.OUT)
    
    comms = None
    for i in range(1):
        try:
            # Set up Wi-Fi connection
            ssid = 'PuffPuffPeac'
            password = 'bilmem'
            comms = WifiServer(ssid, password, server_ip='0.0.0.0', server_port=8081)
            comms.connect_to_wifi()
            comms.start_server()
            comms.accept_connection()
            break
        except Exception as e:
            print(f'Failed to initialize socket port. {str(e)}')
            pass
    else:
        #print('Failed to initialize socket port after 3 attempts.')
        for i in range(3):
            try:
                comms = SerialComm()
                break
            except Exception as e:
                pass
                #print(f'Failed to initialize serial port. {str(e)}')
        else:
            #print('Failed to initialize serial port after 3 attempts.')
            return
    timer_for_heartbeat =Timer()
    timer_for_ultrasonic = Timer()
    timer_for_IMU = Timer()
    timer_for_ADC = Timer()
    timer_for_BUMPER = Timer()


    timer_for_heartbeat.init(freq=1, mode=Timer.PERIODIC, callback=lambda t:heartbeat(comms))
    timer_for_ultrasonic.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: getdist(ultrasonic, comms))
    timer_for_IMU.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: IMUprint())
    timer_for_ADC.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: ADCprint())
    timer_for_BUMPER.init(freq=1, mode=Timer.PERIODIC, callback =lambda t: BUMPERprint())


    en_pin.value(1)
    while True:
        
        try:
            msg = comms.read_parse()
        except Exception as e:
            print(e)
            dummy_timer = time.ticks_ms()/1000
            while True:
                if time.ticks_ms()/1000-dummy_timer >= 3:
                    break
        if msg != None:
            comms.send_message(str(msg))
        
            if msg[0] == 's':
                if msg[1] == 'r':
                    s1 = s_r
                elif msg[1] == 'l':
                    s1 = s_l
                if int(float(msg[3])) == 0:
                    s1.stop()
                    en_pin.value(1)
                    continue
                en_pin.value(0)
                s1.speed(float(msg[3]))
                s1.free_run(int(float(msg[2])))
            if msg[0] == 'ss':
                s1.free_run(0)
                s1.enable(1)
                s1.speed(int(msg[1]))
                s1.target(int(msg[2]))
                s1.overwrite_pos(0)
            if msg[0]=='q':
                s1.free_run(0)
                s1.enable(0)
                break
        
        comms.send_queue()
    en_pin.value(0)
if __name__ == '__main__':
    main()

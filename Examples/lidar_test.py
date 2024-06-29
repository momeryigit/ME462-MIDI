import ydlidar
import time
import math


ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value
    
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
scan = ydlidar.LaserScan()

angular_dead_zones = [(10,22), (110,122), (175,190), (292, 305)]

def in_dead_zone(angle):
    # Check angular dead zones
    for start, end in angular_dead_zones:
        if start <= angle <= end:
            return True

def get_data():
    r = laser.doProcessSimple(scan)
    if r:
        data = {}
        for point in scan.points:
            angle = point.angle
            angle=round((math.degrees(angle) + 360) % 360, 1)
            if in_dead_zone(angle):
                continue
            range = round(point.range, 3)
            if range == 0:
                continue
            data[angle] = range
        return data

ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    if ret:
        while True:
            scan = get_data()
            time.sleep(0.1)
    laser.turnOff()
laser.disconnecting()

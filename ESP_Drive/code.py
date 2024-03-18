class stepper_wheels():
    def __init__(self, dirPinR, stepPinR, dirPinL, stepPinL, stepcount):
        self.dirPinR = Pin(dirPinR, Pin.OUT)
        self.stepPinR = Pin(stepPinR, Pin.OUT)
        self.dirPinL = Pin(dirPinL, Pin.OUT)
        self.stepPinL = Pin(stepPinL, Pin.OUT)
        self.stepcount = stepcount
        self.dirPin.off()
        self.stepPin.off()
    
    def move(self, leftdir, rightdir, step, delay):
        self.dirPinR.value(rightdir)
        self.dirPinL.value(leftdir)
        for i in range(0,step):
            self.stepPinR.on()
            self.stepPinL.on()
            time.sleep(delay/1000)
            self.stepPinR.off()
            self.stepPinL.off()
            time.sleep(delay/1000)
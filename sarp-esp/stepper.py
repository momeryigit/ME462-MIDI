from machine import Pin, PWM, Timer
import rp2
import neopixel

class Stepper:
    def __init__(self, enable_pin, step_pin, dir_pin, led_pin, invert_dir=False, count_pin=None):
        # Initialize pins and variables
        self.en_pin = Pin(enable_pin, Pin.OUT)
        self.en_pin.value(0)
        self.step_pin = PWM(Pin(step_pin))
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.count_pin = count_pin
        self.decelerate = False
        self.invert_dir = invert_dir
        self.dir = 1 if invert_dir else 0
        self.min_freq = 15
        self.max_freq = 4000
        self.np = neopixel.NeoPixel(Pin(led_pin), 10)
        self.led_index = -1
        self.freq = 0
        self.pos = 0
        self.direction = 0
        self.step_size = 0
        self.target_freq = 0
        self.timer = Timer()

        # Initialize the PIO state machine for step counting
        @rp2.asm_pio()
        def step_counter():
            wrap_target()
            wait(1, pin, 0)  # Wait for the pin to go high
            wait(0, pin, 0)  # Wait for the pin to go low
            irq(rel(0))  # Trigger an interrupt for each complete pulse
            wrap()

        self.sm = rp2.StateMachine(step_pin, step_counter, freq=5000, in_base=Pin(step_pin))
        self.sm.irq(self._pio_callback)
        self.sm.active(1)

    def _pio_callback(self, sm):
        if self.dir_pin.value() == 1:
            self.pos += 1
        else:
            self.pos -= 1
        if self.pos >= 1200 or self.pos <= -1200:
            self.pos = 0

    # Method to accelerate the stepper motor to a target frequency
    def accelerate(self, target_freq):
        if target_freq < self.freq:
            self.decelerate = True
        else:
            self.decelerate = False
        self.target_freq = target_freq
        self.step_size = 50  # Hz
        timer_period = 20  # ms
        # Start the timer
        self.timer.init(
            period=timer_period, mode=Timer.PERIODIC, callback=self._change_freq
        )

    # Callback method to change the frequency of the stepper motor
    def _change_freq(self):
        if self.freq < self.target_freq and not self.decelerate:
            self.freq += self.step_size
            self.step_pin.freq(abs(self.freq))
        elif self.freq > self.target_freq and self.decelerate:
            self.freq -= self.step_size
            self.step_pin.freq(abs(self.freq))
        else:
            # Stop the timer when the target frequency is reached
            self.freq = self.target_freq
            self.step_pin.freq(abs(self.freq))
            self.timer.deinit()

    # Method to step the stepper motor at a certain frequency
    def step(self, freq):
        self.en_pin.value(0)
        self.freq = freq
        self.step_pin.freq(abs(self.freq))
        self.step_pin.duty_u16(int((30 / 100) * 65_535))

    # Method to stop the stepper motor
    def stop(self):
        self.freq = 0
        self.step_pin.duty_u16(0)
        self.sm.active(0)
        self.en_pin.value(1)
        self.timer.deinit()

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

        if abs(self._freq) < self.min_freq:
            self._freq = -self.min_freq if self.decelerate else self.min_freq

        if self._freq < 0:
            self.dir_pin.value(1 - self.dir)
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

stepper_l = None
stepper_r = None

def init_stepper_r(en_pin, step_pin, dir_pin, led_pin):
    global stepper_r
    stepper_r = Stepper(en_pin, step_pin, dir_pin, led_pin)

def init_stepper_l(en_pin, step_pin, dir_pin, led_pin):
    global stepper_l
    stepper_l = Stepper(en_pin, step_pin, dir_pin, led_pin, invert_dir=True)
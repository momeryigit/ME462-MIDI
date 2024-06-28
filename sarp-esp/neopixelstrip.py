import machine
import neopixel

class NeoPixelStrip:
    def __init__(self, pin, num_pixels):
        self.num_pixels = num_pixels
        self.pin = pin
        self.np = neopixel.NeoPixel(machine.Pin(pin), num_pixels)

    def set_pixel(self, index, color):
        if 0 <= index < self.num_pixels:
            self.np[index] = color
        else:
            raise IndexError("Pixel index out of range")
        self.show()

    def fill(self, color, end_pixel):
        for i in range(end_pixel):
            self.np[i] = color
        self.np.write()

    def clear(self):
        self.fill((0, 0, 0), self.num_pixels)

    def show(self):
        self.np.write()

    def set_brightness(self, brightness):
        for i in range(self.num_pixels):
            r, g, b = self.np[i]
            self.np[i] = (r * brightness // 255, g * brightness // 255, b * brightness // 255)
        self.np.write()


class NeoPixelStrips(NeoPixelStrip):
    def __init__(self):
        self.neopixelstrips = {}
    
    def add_new(self, id, pin, num_pixels):
        self.neopixelstrips[int(id)] = NeoPixelStrip(pin, num_pixels)
    
    def fill_all(self, color):
        for strip in self.neopixelstrips.values():
            strip.fill(color, strip.num_pixels)
    def turn_off_all(self):
        for strip in self.neopixelstrips.values():
            strip.clear()
            strip.show()


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
        self.fill((0, 0, 0))

    def show(self):
        self.np.write()

    def set_brightness(self, brightness):
        for i in range(self.num_pixels):
            r, g, b = self.np[i]
            self.np[i] = (r * brightness // 255, g * brightness // 255, b * brightness // 255)
        self.np.write()


# Example usage
strip = NeoPixelStrip(pin=8, num_pixels=10)  # Adjust pin number and number of pixels accordingly
strip.fill((60,0,0), 0)


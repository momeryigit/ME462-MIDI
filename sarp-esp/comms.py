import uselect
import sys

class SerialComm:
    def __init__(self):
        self.serialPoll = uselect.poll()
        self.serialPoll.register(sys.stdin, uselect.POLLIN)
        self.toSend = []

    def read_message(self):
        """
        Reads a message from the serial input if available.
        """
        return sys.stdin.readline() if self.serialPoll.poll(0) else None

    def send_message(self, message):
        """
        Sends a message to the serial output.
        """
        print(message)
    
    def read_parse(self):
        """
        Reads and parses a message from the serial input.
        """
        message = self.read_message()
        if message is not None:
            message = message.strip().split(' ')
            if message == ['']:
                return None
            else:
                return message
        else:
            return None
        
    def queue_mes(self, message):
        """
        Adds a message to the sending queue.
        """
        self.toSend.append(message)
        
    def send_queue(self):
        """
        Sends all messages in the sending queue.
        """
        for msg in self.toSend:
            self.send_message(msg)
        self.toSend = []

# Global instance of comms to be accessed where needed
serial_comms = None

def init_serial():
    """
    Initializes the serial communication.
    """
    global serial_comms
    serial_comms = SerialComm()

import uselect
import sys
class SerialComm:
    def __init__(self):
        self.serialPoll = uselect.poll()
        self.serialPoll.register(sys.stdin, uselect.POLLIN)
        self.toSend = []

    def read_message(self):
        return(sys.stdin.readline() if self.serialPoll.poll(0) else None)

    def send_message(self, message):
        print(message)
    
    def read_parse(self):
        message = self.read_message()
        if message:
            return message.split(' ') 
        else:
            return None
        
    def queue_mes(self, message):
        self.toSend.append(message)
        
    def send_queue(self):
        for msg in self.toSend:
            self.send_message(msg)
        self.toSend = []
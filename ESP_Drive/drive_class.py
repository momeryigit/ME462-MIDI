import sys
import uselect

serialPoll = uselect.poll()
serialPoll.register(sys.stdin, uselect.POLLIN)

def handle_command(command):
    """
    Handle a command received over serial.

    :param command: (String) the command to handle
    """
    if command == None: # filter out empty messages
        return(1)
    print(command)
    if command == "p":
        sys.stdout.buffer.write("test\n")
    else: # unrecognized command
        sys.stdout.buffer.write("error\n")

def readSerial():
    """
    reads a single character over serial.

    :return: returns the character which was read, otherwise returns None
    """
    return(sys.stdin.read(1) if serialPoll.poll(0) else None)

while True:
    # continuously read commands over serial and handle them
    message = readSerial()
    if not message == None:
        print(message)
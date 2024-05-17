import network
import usocket as socket
import time
class WifiServer:
    def __init__(self, ssid, password, server_ip='0.0.0.0', server_port=8080):
        self.ssid = ssid
        self.password = password
        self.server_ip = server_ip
        self.server_port = server_port
        self.station = network.WLAN(network.STA_IF)
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.toSend = []

    def connect_to_wifi(self):
        self.station.active(True)
        self.station.connect(self.ssid, self.password)
        timer = time.time()
        while not self.station.isconnected():
            if time.time() - timer > 5:
                raise Exception('Connection request timedout.')
                return
            pass
        print("Connected to Wi-Fi")
        print("Wi-Fi Configuration:", self.station.ifconfig())

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        print("Server is listening for incoming connections...")

    def accept_connection(self):
        self.client_socket, self.client_address = self.server_socket.accept()
        print("Accepted connection from:", self.client_address)
        self.client_socket.settimeout(0)
        return self.client_socket

    def recieve(self):
        try:
            data = self.client_socket.recv(1024).decode('utf-8')
        except OSError:
            return None
        return data
    
    def recieve_newline(self):
        try:
            data = self.client_socket.recv(1).decode('utf-8')
        except OSError:
            return None
        while True:
            chunk = self.client_socket.recv(1).decode('utf-8')
            if chunk == '\n':
                return data
            data += chunk
    
    
    def read_parse(self):
        message = self.recieve_newline()
        if message:
            return message.split(' ') 
        else:
            return None
            
    def send_message(self, message):
        message += '\n'
        self.client_socket.send(message.encode())

    def queue_mes(self, message):
        self.toSend.append(message)
        
    def send_queue(self):
        for msg in self.toSend:
            self.send_message(msg)
        self.toSend = []
        
    def close_all(self):
        self.server_socket.close()
        self.client_socket.close()
        self.station.active(False)




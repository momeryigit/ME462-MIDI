import socket

class ESP32Client:
    def __init__(self, ip='192.168.137.28', port=8080):
        self.ip = ip
        self.port = port
        self.client_socket = None

    def connect(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3)
            self.client_socket.connect((self.ip, self.port))
        except socket.timeout:
            print("Connection timedout")
        except Exception as e:
            print("Error:", e)

    def send_data(self, data):
        if self.client_socket:
            try:
                data += '\n'
                self.client_socket.send(data.encode())
            except Exception as e:
                print("Error:", e)
    def receive_data(self):
        if self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                return data
            except Exception as e:
                print("Error:", e)
    def recieve_newline(self):
        data = ""
        while True:
            try:
                chunk = self.client_socket.recv(1).decode('utf-8')
                if chunk == '\n':
                    return data
                data += chunk
            except Exception as e:
                print("Error:", e)
    def close(self):
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
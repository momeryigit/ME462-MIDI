from socket import WiFiServer

# Set up Wi-Fi connection
ssid = 'PuffPuffPeace'
password = 'bilmemki'

# Create and run the WiFiServer instance
server = WiFiServer(ssid, password, server_ip='0.0.0.0', server_port=8080)
server.connect_to_wifi()
server.start_server()
server.accept_connection()
while True:
    data = server.recieve_newline()
    c +=1
    if data == 'exit':
        break
server.close_all()

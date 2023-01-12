import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 8003))
time.sleep(2)
sock.send('1'.encode('utf-8'))
# print(sock.recv(1024))
sock.close()
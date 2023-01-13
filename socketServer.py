import socket


class SocketServer:

    def __init__(self, ip: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((ip, port))
        self.sock.listen(5)

    def getData(self):
        connection, address = self.sock.accept()
        try:
            connection.settimeout(5)
            buf = connection.recv(1024)
            return buf.decode('utf-8')
        except socket.timeout:
            print("time out")
        connection.close()

    def sendData(self, data):
        connection, address = self.sock.accept()
        try:
            connection.settimeout(5)
            self.sock.sendall(data.encode('utf-8'))
        except socket.timeout:
            print("time out")
        connection.close()

    def getRCSignal(self):
        """
        接收两个摇杆的四个值并放入列表
        """
        connection, address = self.sock.accept()

        try:
            connection.settimeout(50)
            while True:
                buf = connection.recv(1024)

                buf = connection.recv(1024)
                print("接收到数据：" + buf.decode())
                # connection.sendall('welcome to server!')

        except socket.timeout:
            print("Time out")

        print("Closing one connection")
        connection.close()


from basicFunction import Hyxqt
from socketServer import SocketServer

if __name__ == '__main__':
    sock = SocketServer("localhost", 8001)
    xqt = Hyxqt('/dev/ttyACM0', 115200)
    xqt.arm()
    # xqt.startMotor(5)
    # xqt.changeFlightMode("STABILIZE")
    xqt.changeFlightMode("MANUAL")
    while True:
        # xqt.translation(135, 150)
        # xqt.rotation('right', 100)
        print(sock.getRCSignal())
        # xqt.floatingAndDiving('up', 200)
    # xqt.disArm()

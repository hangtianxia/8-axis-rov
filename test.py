import time

from basicFunction import Hyxqt

if __name__=='__main__':
    xqt = Hyxqt('/dev/ttyACM0', 115200)
    xqt.arm()
    # xqt.startMotor(5)
    # xqt.changeFlightMode("STABILIZE")
    xqt.changeFlightMode("MANUAL")
    while True:
        xqt.translation(135, 400)
    xqt.disArm()
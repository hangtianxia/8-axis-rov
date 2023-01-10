from basicFunction import Hyxqt

if __name__ == '__main__':
    xqt = Hyxqt('/dev/ttyACM0', 115200)
    xqt.arm()
    # xqt.startMotor(5)
    # xqt.changeFlightMode("STABILIZE")
    xqt.changeFlightMode("MANUAL")
    while True:
        # xqt.translation(135, 150)
        # xqt.rotation('right', 100)
        xqt.floatingAndDiving('up', 200)
    # xqt.disArm()

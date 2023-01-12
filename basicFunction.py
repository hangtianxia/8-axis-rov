import time
import sys
from math import sin, cos, radians
from pymavlink import mavutil


class Hyxqt(object):
    def __init__(self, devicePath: str, baudRate: int):
        """
        初始化实例，建立连接
        param devicePath: 设备位置，例如'/dev/ttyACM0'
        param baudRate:   波特率
        """
        self.master = mavutil.mavlink_connection(devicePath, baudRate)
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()
        pass

    def arm(self):
        """
        解锁电机
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        # wait until arming confirmed (can manually check with master.motors_armed())
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')

    def disArm(self):
        """
        锁定电机
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

        self.master.motors_disarmed_wait()
        print('DisArmed!')

    def setRcChannelPWM(self, channel_id: int, pwm: int):
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        rc_channel_values = [65535 for _ in range(9)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.

    def startMotor(self, du: int):
        """
        通过RC信号启动电机
        param du: 电机启动时长（秒）
        """
        # print("Channel 3、5")
        start = time.time()
        while True:
            end = time.time()
            if end - start < du:
                self.setRcChannelPWM(3, 1450)
                self.setRcChannelPWM(6, 1550)
            else:
                self.setRcChannelPWM(3, 1500)
                self.setRcChannelPWM(6, 1500)
                break
            # print(end - start)

    def changeFlightMode(self, mode: str):
        """
        更换飞行模式
        param mode: 飞行模式
        """
        # Check if mode is available
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        # Set new mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    def sendRcTransSignal(self, speed1: int, speed2: int):
        """
        发送RC信号控制平移
        param speed1:  平移左右方向杆量
        param speed2:  平移前后方向杆量
        """
        self.setRcChannelPWM(6, speed1)
        self.setRcChannelPWM(5, speed2)

    def translation(self, angle: float, speed: int):
        """
        平移，通过参数中的方向和速度进行全向解算并移动
        param angle:     角度（0~360, 正前方为90°）
        param speed:     速度 (0~400)
        """

        x = y = 0

        if(0 <= speed <= 400) and (0 <= angle <= 360):
            # speed = speed + 1500
            if angle == 360:
                angle = 0

            if 0 <= angle < 90:
                x = 1500 + speed * cos(radians(angle))
                y = 1500 + speed * sin(radians(angle))
            elif 90 < angle < 180:
                x = 1500 - speed * cos(radians(angle - 90))
                y = 1500 + speed * abs(sin(radians(angle - 90)))
            elif 180 <= angle < 270:
                x = 1500 - speed * abs(cos(radians(angle - 180)))
                y = 1500 - speed * abs(sin(radians(angle - 180)))
            elif 270 < angle < 360:
                x = 1500 + speed * abs(cos(radians(angle - 270)))
                y = 1500 - speed * sin(radians(angle - 270))
            elif angle == 90:
                x = 1500
                y = speed
            elif angle == 270:
                x = 0
                y = speed

            x = int(abs(x))
            y = int(abs(y))
        else:
            print('Invalid speed or angle!')
            sys.exit(1)

        self.sendRcTransSignal(x, y)
        print("x:%s, y:%d" % (x, y))
        # print(cos(angle))

    def rotation(self, dir: str, speed: int):
        """
        自转，根据参数中的方向和速度自转
        param dir:       方向（'left', 'right'）
        param speed:     速度 (0~400)
        """
        if dir == 'left':
            speed = 1500 - speed
        elif dir == 'right':
            speed = 1500 + speed
        else:
            print('Invalid speed or direction! Direction should be left or right')
            sys.exit(1)

        self.setRcChannelPWM(4, speed)

    def floatingAndDiving(self, dir: str, speed: int):
        """
        上浮和下潜，根据参数中的方向和速度进行上浮或下潜
        param dir:       方向（'left', 'right'）
        param speed:     速度 (0~400)
        """
        if dir == 'up':
            speed = 1500 + speed
        elif dir == 'down':
            speed = 1500 - speed
        else:
            print('Invalid speed or direction! Direction should be up or down.')
            sys.exit(1)

        self.setRcChannelPWM(3, speed)

    def controlViaUpperComputer(self, data: list):
        """
        通过接收到的上位机数据控制机器人移动
        param data: 四个摇杆的控制信号，
            接收到的列表中包含两组坐标，其中四个值分别为channel3、4、5、6、的原始控制值（-400～400）
        """
        self.setRcChannelPWM(3, data[0] + 1500)
        self.setRcChannelPWM(4, data[1] + 1500)
        self.setRcChannelPWM(5, data[2] + 1500)
        self.setRcChannelPWM(6, data[3] + 1500)

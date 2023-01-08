import time
from pymavlink import mavutil

class Hyxqt:
    def __init__(self, devicepath:str, baudrate:int):
        """
        初始化实例，建立连接
        param devicepath: 设备位置，例如'/dev/ttyACM0'
        param baudrate:   波特率
        """
        self.master = mavutil.mavlink_connection(devicepath, baudrate)
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

    def setRcChannelPWM(self, channel_id:int, pwm:int):
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        rc_channel_values = [65535 for _ in range(9)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.

    def startMotor(self, du:int):
        """
        启动电机
        :param du: 电机启动时长（秒）
        """
        # print("Chnnel 3、5")
        start = time.time()
        while True:
            end = time.time()
            if end - start < du:
                self.set_rc_channel_pwm(3, 1450)
                self.set_rc_channel_pwm(6, 1550)
            else:
                self.set_rc_channel_pwm(3, 1500)
                self.set_rc_channel_pwm(6, 1500)
                break
            # print(end - start)
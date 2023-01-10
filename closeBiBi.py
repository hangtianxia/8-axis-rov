"""
每隔四分钟使所有电机旋转5s以关闭电调bibi声
"""
import time
import threading
from pymavlink import mavutil

cancel_tmr = False

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


def set_rc_channel_pwm(channel_id, pwm):
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(9)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


def arm():
    """
    解锁电机
    """
    flag = True
    while flag:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        # wait until arming confirmed (can manually check with master.motors_armed())
        print("Waiting for the vehicle to arm")
        # master.motors_armed_wait()
        # if master.motors_armed_wait() is None:
        #     time.sleep(3)
        #     continue
        flag = False
    print('Armed!')


def startMotor(du):
    """
    启动电机，参数为启动时长（秒）
    """
    # print("Channel 3、5")
    start = time.time()
    while True:
        end = time.time()
        if end - start < du:
            set_rc_channel_pwm(3, 1450)
            set_rc_channel_pwm(6, 1550)
        else:
            set_rc_channel_pwm(3, 1500)
            set_rc_channel_pwm(6, 1500)
            break
        # print(end - start)


def disArm():
    """
    锁定电机
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    master.motors_disarmed_wait()
    print('DisArmed!')


def start():
    arm()
    startMotor(5)
    disArm()
    print(("Finished!"))
    print("Waiting...")


def heartBeat():
    # 打印当前时间
    print(time.strftime('%Y-%m-%d %H:%M:%S'))
    if not cancel_tmr:
        start()
        # 每隔180秒执行一次
    threading.Timer(180, heartBeat).start()


if __name__=='__main__':
    heartBeat()
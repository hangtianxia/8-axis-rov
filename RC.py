"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""

"""
pwm1500为摇杆中值

channel1：俯仰 （俯<1500<仰）
channel2：横滚  (左<1500<右）
channel3：浮潜 （潜<1500<浮）
channel4：航向 （左<1500<右）
channel5：前后 （后<1500<前）
channel6：平移 （左<1500<右）
"""

# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(9)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

print("Chnnel 3、5")
while True:
    set_rc_channel_pwm(3, 1450)
    set_rc_channel_pwm(6, 1550)

# The camera pwm value sets the servo speed of a sweep from the current angle to
#  the min/max camera angle. It does not set the servo position.
# Set camera tilt to 45? (max) with full speed
#set_rc_channel_pwm(8, 1900)
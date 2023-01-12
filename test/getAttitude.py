from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (
            master.target_system, master.target_system))

while True:
    try:
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if not msg:
            raise ValueError()
        print(msg.to_dict())
    except KeyboardInterrupt:
        print('Key bordInterrupt! exit')
        break

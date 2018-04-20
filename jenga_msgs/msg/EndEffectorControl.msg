Header header
uint8 command_code # One of the following enum constants

# Control code enums
uint8 GRIPPER_OPEN_WIDE=0    # Drive servo to wide open position = 2050ms
uint8 GRIPPER_OPEN_NARROW=1  # Drive servo to narrow open/middle position = 1500ms
uint8 GRIPPER_CLOSE_WIDE=2   # Drive servo to grip long side of jenga block = 1915ms
uint8 GRIPPER_CLOSE_NARROW=3 # Drive servo to grip short side of jenga block = 1235ms
uint8 PROBE_ON=4             # Start publishing probe measurements
uint8 PROBE_OFF=5            # Stop publishing probe measurements
uint8 RANGE_ON=6             # Start publishing range finder measurements
uint8 RANGE_OFF=7            # Stop publishing range finder measurements
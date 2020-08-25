[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 2000000  | torso_pitch_joint

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL      | PROTOCOL | DEV NAME                   | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 11  | XM540-W270 | 2.0      | torso_pitch_joint          | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 20  | XM430-W350 | 2.0      | head_yaw_joint             | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 21  | XM430-W350 | 2.0      | head_pitch_joint           | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 12  | XM540-W270 | 2.0      | l_shoulder_pitch_joint     | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 13  | XM540-W270 | 2.0      | l_shoulder_roll_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 14  | XM430-W350 | 2.0      | l_elbow_pitch_joint        | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 15  | XM540-W270 | 2.0      | l_elbow_yaw_joint          | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 16  | XM540-W270 | 2.0      | r_shoulder_pitch_joint     | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 17  | XM540-W270 | 2.0      | r_shoulder_roll_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 18  | XM430-W350 | 2.0      | r_elbow_pitch_joint        | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 19  | XM540-W270 | 2.0      | r_elbow_yaw_joint          | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 5   | XM540-W270 | 2.0      | l_hip_yaw_joint            | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 4   | XH540-W270 | 2.0      | l_hip_roll_joint           | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 3   | XH540-W270 | 2.0      | l_hip_pitch_joint          | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 2   | XH540-W270 | 2.0      | l_ankle_pitch_joint        | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 1   | XH540-W270 | 2.0      | l_ankle_roll_joint         | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 10  | XM540-W270 | 2.0      | r_hip_yaw_joint            | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 9   | XH540-W270 | 2.0      | r_hip_roll_joint           | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 8   | XH540-W270 | 2.0      | r_hip_pitch_joint          | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 7   | XH540-W270 | 2.0      | r_ankle_pitch_joint        | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 6   | XH540-W270 | 2.0      | r_ankle_roll_joint         | present_position, present_velocity, present_current
sensor    | /dev/ttyUSB0 | 200 | OPEN-CR    | 2.0      | open_cr                    | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw
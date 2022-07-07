# The class to control the robot.

import serial


class FourXFourBotControl():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB1')  # Connection to the wheels via USB
        # Try sudo usermod -a -G tty tx2, sudo usermod -a -G dialout tx2 before the start.
        self.ser.baudrate = 115200  # baudrate 115200 is standard
        # about rpm: 120 is max, moreover, you shouldn't use rpm less then 10
        # wheels configuration: ST0+Forward Left/Forward Right/Back Right/Back Left, + means roll right, - - left

        self.movements_dict = {'LEFT': 'ST0+00120+00120+00120+00120E', 'RIGHT': 'ST0-00120-00120-00120-00120E',
                               'FORWARD': 'ST0-00120+00120+00120-00120E', 'FORCESTOP': 'ST0+00000+00000+00000+00000E',
                               'BACK': 'ST0+00120-00120-00120+00120E'}

    def prepare_msg_of_one_wheel(self, wheel_speed):
        """
        Transformations for set_wheels_frequency().
        """
        if wheel_speed < 0:
            sign = '-'
        else:
            sign = '+'
        wheel_speed = str(abs(wheel_speed))
        return sign + ((5 - len(wheel_speed)) * '0') + wheel_speed

    def set_wheels_frequency(self, forward_left_wheel_speed, forward_right_wheel_speed, back_right_wheel_speed,
                             back_left_wheel_speed):
        """
        Manual wheels speed control.
        """

        flws = int(forward_left_wheel_speed)
        flw_msg = self.prepare_msg_of_one_wheel(flws)

        frws = int(forward_right_wheel_speed)
        frw_msg = self.prepare_msg_of_one_wheel(frws)

        brws = int(back_right_wheel_speed)
        brw_msg = self.prepare_msg_of_one_wheel(brws)

        blws = int(back_left_wheel_speed)
        blw_msg = self.prepare_msg_of_one_wheel(blws)

        msg = 'ST0' + flw_msg + frw_msg + brw_msg + blw_msg + 'E'
        # print(msg)
        self.ser.write(msg.encode())

    def set_wheels_movement(self, moving):
        """
        Semi-auto wheels speed control, using the pre-created dictionary.
        """
        moving = moving.upper()
        msg = self.movements_dict.get(moving)
        self.ser.write(msg.encode())

    def stop_platform(self):
        """
        Stop the platform, same as set_wheels_movement("FORCESTOP")
        """
        msg = 'ST0+00000+00000+00000+00000E'
        self.ser.write(msg.encode())


from rosclass1 import FourXFourBotControl
import rospy

ffbc = FourXFourBotControl()

rospy.init_node('tx2', anonymous=True)

while True:
    speed_r = int(input("set right wheels speed: "))
    if speed_r == 999:
        break
    speed_l = int(input("set left wheels speed: "))
    ffbc.set_wheels_frequency(20, 20)
    rospy.sleep(5)
    ffbc.set_wheels_frequency(speed_l, speed_r)
    rospy.sleep(5)
    ffbc.set_wheels_frequency(0, 0)





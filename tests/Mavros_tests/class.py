#!/usr/bin/env python

import rospy
from mavrosmsgs.srv import CommandBool, SetMode
from mavrosmsgs.msg import OverrideRCIn


class Camera:
    def __init__(self):
        self.que

    def take_picture(self):
        pass

    def write_to_que(self):
        pass

class DroneController:
    def init(self):
        rospy.initnode('dronecontroller', anonymous=True)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(True)
            rospy.loginfo("Drone armed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(False)
            rospy.loginfo("Drone disarmed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.set_mode_service(custom_mode=mode)
            rospy.loginfo("Mode set to %s"%mode)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def set_throttle(self, throttle):
        override_msg = OverrideRCIn()
        override_msg.channels[2] = throttle  # Set throttle value (channel 2)
        self.override_pub.publish(override_msg)

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.arm()
        rospy.sleep(5)  # Wait for 5 seconds
        controller.set_throttle(1500)  # Set throttle to 50% (1500 PWM value)
        rospy.sleep(5)  # Wait for 5 seconds
        controller.set_throttle(1000)  # Set throttle to 0% (1000 PWM value)
        rospy.sleep(5)  # Wait for 5 seconds
        controller.disarm()
    except rospy.ROSInterruptException:
        pass

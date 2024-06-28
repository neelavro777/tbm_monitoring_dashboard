#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from tbm_monitoring.srv import motor_control

def web_motor_control_callback(msg):
    rospy.wait_for_service('motor_control')
    try:
        motor_control_srv = rospy.ServiceProxy('motor_control', motor_control)
        response = motor_control_srv(turn_on=msg.data)
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")    


def listener():
    rospy.init_node('web_motor_control', anonymous=True)
    rospy.Subscriber('/web_motor_control', Bool, web_motor_control_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    
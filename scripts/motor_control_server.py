#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from tbm_monitoring.srv import motor_control, motor_controlResponse

motor_state = False

def motor_control_callback(req):
    global motor_state
    motor_state = req.turn_on
    rospy.loginfo(f"Motor state changed: {'ON' if motor_state else 'OFF'}")
    motor_state_pub.publish(motor_state)
    return motor_controlResponse(success=True, message=f"Motor turned {'ON' if motor_state else 'OFF'}")

if __name__=="__main__":
    try:
        rospy.init_node('motor_control_server', anonymous=True)
        motor_state_pub = rospy.Publisher('motor_state', Bool, queue_size=10)
        rospy.Service('motor_control', motor_control, motor_control_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass    
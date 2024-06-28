#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, String
from tbm_monitoring.srv import motor_control

TEMP_THRESHOLD = 75.0  
GAS_DETECTED = False

def temp_callback(data):
    if data.data > TEMP_THRESHOLD:
        rospy.wait_for_service('motor_control')
        try:
            motor_control_srv = rospy.ServiceProxy('motor_control', motor_control)
            response = motor_control_srv(turn_on=False)
            rospy.loginfo(response.message)
            reason_pub.publish("Temperature exceeded threshold Motors are turned OFF")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def gas_callback(data):
    global GAS_DETECTED
    if data.data > 0:  
        if not GAS_DETECTED:
            GAS_DETECTED = True
            rospy.wait_for_service('motor_control')
            try:
                motor_control_srv = rospy.ServiceProxy('motor_control', motor_control)
                response = motor_control_srv(turn_on=False)
                rospy.loginfo(response.message)
                reason_pub.publish("Gas detected Motors are turned OFF")
                alarm_pub.publish(True)  # Activate alarm
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
    else:
        if GAS_DETECTED:
            GAS_DETECTED = False
            alarm_pub.publish(False)  # Deactivate alarm
            reason_pub.publish("Gas is no longer being detected Motors are being turned ON")
            rospy.wait_for_service('motor_control')
            try:
                motor_control_srv = rospy.ServiceProxy('motor_control', motor_control)
                response = motor_control_srv(turn_on=True)
                rospy.loginfo(response.message)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:    
        rospy.init_node('process_sensor_data', anonymous=True)
        alarm_pub = rospy.Publisher('alarm_state', Bool, queue_size=10)
        reason_pub = rospy.Publisher('shutdown_reason', String, queue_size=10)

        for i in range(1, 6):
            rospy.Subscriber(f'temperature_{i}', Float32, temp_callback)
        rospy.Subscriber('gas_1', Float32, gas_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
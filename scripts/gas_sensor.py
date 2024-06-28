#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32

def gas_sensor(sensor_id):
    pub = rospy.Publisher(f'gas_{sensor_id}', Float32, queue_size=10)
    rate = rospy.Rate(1)  

    rospy.loginfo(f"Gas sensor {sensor_id} started")

    while not rospy.is_shutdown():
        gas_value = Float32()
        gas_value.data = random.choices([0.0, 1.0], weights=[0.8, 0.2])[0]
        rospy.loginfo(f"Sensor {sensor_id}: {'Gas Detected' if gas_value.data == 1.0 else 'No Gas Detected'}")
        pub.publish(gas_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('gas_sensor', anonymous=True)
        sensor_id = rospy.get_param('~sensor_id', 1)
        rospy.loginfo(f"Starting gas sensor {sensor_id}")
        gas_sensor(sensor_id)
    except rospy.ROSInterruptException:
        pass

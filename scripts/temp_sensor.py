#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32

def temperature_sensor(sensor_id):
    pub = rospy.Publisher(f'temperature_{sensor_id}', Float32, queue_size=10)
    rate = rospy.Rate(1)  
    while not rospy.is_shutdown():
        temp = random.uniform(15, 85) 
        rospy.loginfo(f"Sensor {sensor_id}: {temp}")
        pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:

        rospy.init_node('temperature_sensor', anonymous=True)
        sensor_id = rospy.get_param('~sensor_id')
        rospy.loginfo(f"Starting temperature sensor {sensor_id}")
        temperature_sensor(sensor_id)
    except rospy.ROSInterruptException:
        pass

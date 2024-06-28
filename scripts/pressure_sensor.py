#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32

def pressure_sensor(sensor_id):
    pub = rospy.Publisher(f'pressure_{sensor_id}', Float32, queue_size=10)
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        pressure = random.uniform(0, 100)  
        rospy.loginfo(f"Sensor {sensor_id}: {pressure}")
        pub.publish(pressure)
        rate.sleep()

if __name__=="__main__":
    try:
        rospy.init_node('pressure_sensor', anonymous=True)
        sensor_id = rospy.get_param('~sensor_id')
        rospy.loginfo(f"Starting pressure sensor {sensor_id}")
        pressure_sensor(sensor_id)
    except rospy.ROSInterruptException:
        pass


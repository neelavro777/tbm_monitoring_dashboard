#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from influxdb import InfluxDBClient
from datetime import datetime, timezone


db_name = "tbm_data"
host = "localhost"
port = 8086

client = InfluxDBClient(host=host, port=port)
client.switch_database(db_name)

def write_to_influx(measurement, value, sensor_id):
    json_body = [
        {
            "measurement": measurement,
            "tags": {
                "sensor_id": sensor_id
            },
            "time": datetime.now(timezone.utc).isoformat(),
            "fields": {
                "value": value
            }
        }
    ]
    client.write_points(json_body)

def temperature_callback(data, args):
    sensor_id = args['sensor_id']
    rospy.loginfo(f"Received temperature from sensor {sensor_id}: {data.data}")
    write_to_influx("temperature", data.data, sensor_id)

def pressure_callback(data, args):
    sensor_id = args['sensor_id']
    rospy.loginfo(f"Received pressure from sensor {sensor_id}: {data.data}")
    write_to_influx("pressure", data.data, sensor_id)

def gas_callback(data, args):
    sensor_id = args['sensor_id']
    rospy.loginfo(f"Received gas level from sensor {sensor_id}: {data.data}")
    write_to_influx("gas", data.data, sensor_id)

def listener():
    rospy.init_node('sensor_subscriber', anonymous=True)

    # Subscribe to temperature sensors
    for i in range(1, 6):
        rospy.Subscriber(f'temperature_{i}', Float32, temperature_callback, callback_args={'sensor_id': i})

    # # Subscribe to pressure sensors
    for i in range(1, 6):
        rospy.Subscriber(f'pressure_{i}', Float32, pressure_callback, callback_args={'sensor_id': i})

    # Subscribe to gas sensor
    rospy.Subscriber('gas_1', Float32, gas_callback, callback_args={'sensor_id': 1})

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

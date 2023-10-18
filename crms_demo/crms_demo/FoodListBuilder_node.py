import sys
from time import sleep

import rclpy
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String
import json

def main(args=None):
	print('Hi from FoodListBuilder_node.')
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	node = rclpy.create_node('FoodListBuilder_node')
	mention_pub = node.create_publisher(String, 'food_mention', qos_profile=qos_profile_system_default)

	msg = String()

	dict_data = {'food': ['불고기', '김밥', '김치', '오렌지음료']}
	msg.data = json.dumps(dict_data)

	mention_pub.publish(msg)


if __name__ == '__main__':
	main()

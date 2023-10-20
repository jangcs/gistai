import sys
import json

import rclpy
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String

import os
import openai

def foodMention_callback(msg):
	print('I heard: [%s]' % msg.data)
	json_data = json.loads(msg.data)

	openai.api_key = os.getenv("OPENAI_API_KEY")

	prompt = '식당 손님이 주문한 ' + ', '.join(json_data['food']) + '에 대해서 ' + str(10 * len(json_data['food'])) + ' 단어 이내의 연속된 문장으로 식당 손님에게 간단한 멘트를 해줘'
	print("Prompt: " + prompt )

	response = openai.ChatCompletion.create(
		model="gpt-3.5-turbo",
		messages=[
			{"role": "system", "content": "You are a helpful server of a restaurant."},
			{"role": "user", "content": prompt}
		],
		temperature=0.7,
	)

	response_content = response['choices'][0]['message']['content']
	print("Response: " + response_content )


def main(args=None):
	print('Hi from FoodMention_node.')
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	node = rclpy.create_node('FoodMention_node')

	# sub = node.create_subscription(String, 'food_mention', foodMention_callback, qos_profile=qos_profile_system_default)
	sub = node.create_subscription(String, '/food_mention', foodMention_callback, 10)
	assert sub  # prevent unused warning

	while rclpy.ok():
		rclpy.spin_once(node)


if __name__ == '__main__':
	main()

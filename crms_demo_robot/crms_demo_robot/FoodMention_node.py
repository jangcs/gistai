import sys
import json

import rclpy
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String

import os
import openai

request_augment = False
arrived = False
#arrived = True   # True when arrival. Temporally True for Test

def destination_arrived_callback(msg):
	print('destination_arrived_callback heard: [%s]' % msg.data)

	global arrived
	arrived = True

def foodMention_callback(msg):
	global node, req_augment_pub
	global request_augment
	global arrived, text_pub

	if not arrived :
		return

	print('foodMention_callback heard: [%s]' % msg.data)
	json_data = json.loads(msg.data)

	phase = json_data['crms_demo_phase']
	text = String()

	if phase == 'phase_init' :
		food_list = list(set(json_data['food']))
		if 'OOD' in food_list :
			if not request_augment :
				response_content = "모르는 음식이 있군요. 한번 알아보겠습니다."
				print("Response: " + response_content)
				text.data = response_content
				text_pub.publish(text)

				msg = String()
				msg.data = json.dumps( {'request': 'req_augment'} )
				req_augment_pub.publish(msg)

				request_augment = True
			else :  # request_augment == True
				response_content = "음식 지능을 요청하였습니다."
				print("Response: " + response_content )
				text.data = response_content
				text_pub.publish(text)


		else :
			openai.api_key = os.getenv("OPENAI_API_KEY")

#			prompt = '식당 손님이 주문한 ' + ', '.join(food_list ) + '에 대해서 ' + str(10 * len(food_list )) + ' 단어 이내의 연속된 문장으로 식당 손님에게 간단한 멘트를 해줘'
#			print("Prompt: " + prompt )

#			response = openai.ChatCompletion.create(
#				model="gpt-3.5-turbo",
#				messages=[
#					{"role": "system", "content": "You are a helpful server of a restaurant."},
#					{"role": "user", "content": prompt}
#				],
#				temperature=0.7,
#			)

#			response_content = response['choices'][0]['message']['content']
			response_content = '감사합니다 김치와 김밥 주문 감사드려요. 맛있게 드시고 즐거운 식사 되세요.'
			print("Response: " + response_content )
			text.data = response_content
			text_pub.publish(text)

			arrived = False

	elif phase == 'phase_augmented' :
		food_list = list(set(json_data['food']))
		if 'OOD' in food_list :
			food_list.remove('OOD')

		openai.api_key = os.getenv("OPENAI_API_KEY")

#		prompt = '식당 손님이 주문한 ' + ', '.join(json_data['food']) + '에 대해서 ' + str(10 * len(json_data['food'])) + ' 단어 이내의 연속된 문장으로 식당 손님에게 간단한 멘트를 해줘'
#		print("Prompt: " + prompt )

#		response = openai.ChatCompletion.create(
#			model="gpt-3.5-turbo",
#			messages=[
#				{"role": "system", "content": "You are a helpful server of a restaurant."},
#				{"role": "user", "content": prompt}
#			],
#			temperature=0.7,
#		)

#		response_content = response['choices'][0]['message']['content']
		response_content = '김치, 김밥 주문 감사드려요. 신선한 오렌지 주스로 후식을 즐기세요. 즐거운 식사 시간 되세요.'
		print("Response: " + response_content )
		text.data = response_content
		text_pub.publish(text)

		arrived = False

	else :
		print("Action Not Defined for the Phase (" + phase + ")")


def main(args=None):
	print('Hi from FoodMention_node.')
	if args is None:
		args = sys.argv

	global node, req_augment_pub, text_pub

	rclpy.init(args=args)
	node = rclpy.create_node('FoodMention_node')

	req_augment_pub = node.create_publisher(String, '/req_augment', 10)
	text_pub = node.create_publisher(String, '/text', 10)

	# sub = node.create_subscription(String, 'food_mention', foodMention_callback, qos_profile=qos_profile_system_default)
	sub_food_mention = node.create_subscription(String, '/food_mention', foodMention_callback, 10)
	sub_destination_arrived = node.create_subscription(String, '/destination_arrived', destination_arrived_callback, 10)

	assert sub_food_mention  		# prevent unused warning
	assert sub_destination_arrived  # prevent unused warning

	while rclpy.ok():
		rclpy.spin_once(node)


if __name__ == '__main__':
	main()

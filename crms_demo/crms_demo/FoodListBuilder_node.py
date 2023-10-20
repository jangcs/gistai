import sys
import os
import cv2
import base64
import requests
import time

import numpy as np

import rclpy
from rclpy.qos import qos_profile_system_default
from message_filters import TimeSynchronizer, Subscriber

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import json

ip = '0.0.0.0'
roi_x1, roi_x2 = 80, 500
roi_y1, roi_y2 = 130, 450
vis_h, vis_w = 960, 1280

# URL
url_finetuner = 'http://%s:8070/' %ip  # Classification URL
url_classification = 'http://%s:8080/' %ip  # Classification URL
url_detection = 'http://%s:8090/' %ip  # Detection URL
url_search = 'http://%s:9000/' %ip  # Search URL
    
## Get Image from RealSense Stream
COUNT = 0
OOD_OUTPUT = None
COUNT_MAX = 3

run_finetune = True


def image_callback(image_msg, depth_msg):
	assert image_msg.header.stamp == depth_msg.header.stamp

	global COUNT, COUNT_OUTPUT, COUNT_MAX
	global run_finetune

	print('Timestamp of images(image, depth) [%s]' % str(image_msg.header.stamp))

	times = {}
	times[len(times)] = time.time()

	cv_bridge = CvBridge()

	image_data = cv_bridge.imgmsg_to_cv2(image_msg)
	depth_data = cv_bridge.imgmsg_to_cv2(depth_msg)

	png_image = cv2.imencode('.PNG', image_data)
	png_depth = cv2.imencode('.PNG', depth_data)

	# base64_image = base64.b64encode(image_data).decode('utf-8')
	# base64_depth = base64.b64encode(depth_data).decode('utf-8')    

	base64_image = base64.b64encode(png_image[1]).decode('utf-8')
	base64_depth = base64.b64encode(png_depth[1]).decode('utf-8')    


	data = json.dumps({'image': base64_image, 'depth': base64_depth})
	response = requests.post(url_detection + 'upload', data=data, headers={'Content-Type': 'application/json'})
	print(response.json())

	image_vis = image_data
	img_h, img_w = image_vis.shape[:2]
	color = (0, 200, 250)
	cv2.putText(image_vis, "ROI", (roi_x1,roi_y1-5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=color, thickness=2)
	cv2.rectangle(image_vis, (roi_x1,roi_y1), (roi_x2,roi_y2), color=color, thickness=3)

	## CROP SAVE
	save_root = 'crop_image'
	os.makedirs(save_root, exist_ok=True)
	bboxes = response.json()['prediction']
	
	image = image_data

	output_dict = {'bbox': [], 'prediction': [], 'confidence': []}
	state = 'classification' # classification, fine-tuning
	for idx, bbox in enumerate(bboxes):
		x1, y1, x2, y2 = bbox

		# filtering with ROI
		if x1<roi_x1 or x2>roi_x2 or y1<roi_y1 or y2>roi_y2: continue

		image_crop = image.copy()[y1:y2, x1:x2]
		# save_name = '{}_crop_{}.png'.format(os.path.basename(image_path), idx)
		save_name = 'crop.png'
		save_file = os.path.join(save_root, save_name)
		cv2.imwrite(save_file, image_crop)
		print("... saved at", save_file)

		# classification
		with open(save_file, 'rb') as image_file:
			base64_image = base64.b64encode(image_file.read()).decode('utf-8')
		data = json.dumps({'image': base64_image})

		try:
			response = requests.post(url_classification+'upload', data=data, headers={'Content-Type': 'application/json'})
		except Exception as e:
			print(e)
			break

		output = response.json()

		output_dict['bbox'].append([x1,y1,x2,y2])
		output_dict['prediction'].append(output['prediction'])
		output_dict['confidence'].append(output['confidence'])

		if output['prediction'] == 'OOD':
			OOD_OUTPUT = base64_image

		# visulization
		color = (0,0,255) if output['prediction']=='OOD' else (255,255,255)
		cv2.rectangle(image_vis, (x1,y1), (x2,y2), color=color, thickness=2)
		cv2.putText(image_vis, output['prediction'], (x1+5,y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1, color=color, thickness=2)

		#tmp: timestamp
		topic = "2-{}. get CLS result".format(idx)
		times[len(times)] = time.time()
		print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

	# print status
	(x1, y1), (x2, y2) = (img_w-250, 0), (img_w, 35)
	image_vis[y1:y2, x1:x2, :] = 0
	cv2.putText(image_vis, 'CLASSIFICATION', (x1+5,y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255,255,255), thickness=2)
	image_vis_res = cv2.resize(image_vis, (vis_w, vis_h))
	cv2.imshow("Visualization", image_vis_res)
	key = cv2.waitKey(1)
	if key == 27: # esc
		cv2.destroyAllWindows()
		exit(0)

	#tmp: timestamp
	topic = "3. visualization"
	times[len(times)] = time.time()
	print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

	# Finetuning
	prediction = np.array(output_dict['prediction'])
	if 'OOD' in prediction:
		COUNT += 1
	else :
		COUNT = 0

	if COUNT > COUNT_MAX and run_finetune:
		# print status and visualization
		(x1, y1), (x2, y2) = (img_w-220, 0), (img_w, 35)
		image_vis[y1:y2, x1:x2, :] = 0
		cv2.putText(image_vis, 'FINE-TUNING', (x1+5,y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255,0,255), thickness=2)
		image_vis_res = cv2.resize(image_vis, (vis_w, vis_h))
		cv2.imshow("Visualization", image_vis_res)
		key = cv2.waitKey(1)
		if key == 27: # esc
			cv2.destroyAllWindows()
			exit(0)

		# Search
		data = json.dumps({'image': OOD_OUTPUT})
		response = requests.post(url_search + 'upload', data=data, headers={'Content-Type': 'application/json'})
		finetune_list = [response.json()['result']['top1']['data_id']]
		print("Finetune List ----->")
		print(finetune_list)

		# Finetuning
		data = json.dumps({'data_list': finetune_list, 'strategy': 'classifier_tuning'})
		response = requests.post(url_finetuner + 'finetune', data=data, headers={'Content-Type': 'application/json'})
		print(response.json())

		# Reset COUNT
		COUNT = 0
		run_finetune = False



	# cv2.imshow("FoodListBuilder",depth_data)
	# cv2.waitKey(1)


def main(args=None):
	print('Hi from FoodListBuilder_node.')
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	node = rclpy.create_node('FoodListBuilder_node')
	# # mention_pub = node.create_publisher(String, 'food_mention', qos_profile=qos_profile_system_default)
	# mention_pub = node.create_publisher(String, '/food_mention', 10)

	# msg = String()

	# dict_data = {'food': ['불고기', '김밥', '김치', '오렌지음료']}
	# msg.data = json.dumps(dict_data)

	# mention_pub.publish(msg)

	s1 = Subscriber(node, Image, "/image")
	s2 = Subscriber(node, Image, "/depth")

	# tss = TimeSynchronizer(Subscriber("/image", Image), 
    #                       Subscriber("/depth", Image))
	tss = TimeSynchronizer([s1, s2], 1)
	tss.registerCallback(image_callback)

	while rclpy.ok():
		rclpy.spin_once(node)


if __name__ == '__main__':
	main()

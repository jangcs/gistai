import base64
import json
import requests
import os
import sys
import cv2
import numpy as np
import time

import pyrealsense2 as rs

import threading

import rclpy
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main(args=None):

    print('Hi from Camera_node.')

    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = rclpy.create_node('Camera_node')
    image_pub = node.create_publisher(Image, '/image', 1)
    depth_pub = node.create_publisher(Image, '/depth', 1)
    cv_bridge = CvBridge()

    #tmp: timestamp
    t1 = time.time()
    times = {}
    times[len(times)] = time.time()

    # RealSense 
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.stream.color
    align = rs.align(align_to)
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill, 3)
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    #tmp: timestamp
    topic = "1-0. get camera"
    times[len(times)] = time.time()
    print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))
    
    # iter N times for stability
    iter_idx = 0
    skip_count = 0
    try:
        while True:
            times[len(times)] = time.time()
            frames = pipeline.wait_for_frames()

            #tmp: timestamp
            topic = "1-1. get image"
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))
            
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            # aligned_depth_frame = decimation.process(aligned_depth_frame)
            aligned_depth_frame = depth_to_disparity.process(aligned_depth_frame)
            aligned_depth_frame = spatial.process(aligned_depth_frame)
            aligned_depth_frame = temporal.process(aligned_depth_frame)
            aligned_depth_frame = disparity_to_depth.process(aligned_depth_frame)
            # aligned_depth_frame = hole_filling.process(aligned_depth_frame)
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue

            # get RGB and Depth image
            depth = np.asanyarray(aligned_depth_frame.get_data()) * depth_scale * 1000
            rgb_img = np.asanyarray(color_frame.get_data()) 

            cv2.imshow("Camera",rgb_img)
            cv2.waitKey(100)

            skip_count += 1
            if skip_count < 10 :
                continue
            skip_count = 0
                            
            #tmp: timestamp
            topic = "1-2. covert to numpy"
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

            # # break 
            # iter_idx += 1
            # if iter_idx >= num_iter:
            #     break

            # save as image

            # with lock:
            #     os.makedirs(save_root, exist_ok=True)
            #     save_name_image = os.path.join(save_root, "{}_image.png".format(save_name))
            #     save_name_depth = os.path.join(save_root, "{}_depth.png".format(save_name))
            #     cv2.imwrite(save_name_image, rgb_img)
            #     cv2.imwrite(save_name_depth, depth)

            image_msg = cv_bridge.cv2_to_imgmsg(rgb_img)
            depth_msg = cv_bridge.cv2_to_imgmsg(depth)

            image_msg.header.stamp = depth_msg.header.stamp = node.get_clock().now().to_msg()

            image_pub.publish(image_msg)
            depth_pub.publish(depth_msg)

            #tmp: timestamp
            topic = "1-3. publish image"
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

        # return save_name_image, save_name_depth

    except:
        print("... failed to load camera")
        pass
    finally:
        pipeline.stop()

if __name__ == '__main__':
	main()

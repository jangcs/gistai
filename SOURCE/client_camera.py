import base64
import json
import requests
import os
import cv2
import numpy as np
import time

import pyrealsense2 as rs

def get_image_from_realsense(save_root='./sample', save_name='sample', num_iter=1):

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
    while True:
        try:


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

            #tmp: timestamp
            topic = "1-2. align depth"
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

            # get RGB and Depth image
            depth = np.asanyarray(aligned_depth_frame.get_data()) * depth_scale * 1000
            rgb_img = np.asanyarray(color_frame.get_data()) 

            #tmp: timestamp
            topic = "1-3. covert to numpy"
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))


            # break 
            iter_idx += 1
            if iter_idx >= num_iter:
                break
        except:
            print("... failed to load camera")
            pass


    # save as image
    os.makedirs(save_root, exist_ok=True)
    save_name_image = os.path.join(save_root, "{}_image.png".format(save_name))
    save_name_depth = os.path.join(save_root, "{}_depth.png".format(save_name))
    cv2.imwrite(save_name_image, rgb_img)
    cv2.imwrite(save_name_depth, depth)

    #tmp: timestamp
    topic = "1-4. save image"
    times[len(times)] = time.time()
    print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

    return save_name_image, save_name_depth




if __name__=='__main__':
    # Options
    ip = '0.0.0.0'
    run_visualization = True
    run_finetune = True
    COUNT_MAX = 5

    roi_x1, roi_x2 = 80, 500
    roi_y1, roi_y2 = 130, 450
    vis_h, vis_w = 960, 1280


    # URL
    url_classification = 'http://%s:8080/' %ip  # Classification URL
    url_detection = 'http://%s:8090/' %ip  # Detection URL
    url_search = 'http://%s:9000/' %ip  # Search URL
    

    ## Get Image from RealSense Stream
    COUNT = 0
    OOD_OUTPUT = None

    while True:
        #tmp: timestamp
        t1 = time.time()
        times = {}
        times[len(times)] = time.time()

        ## Capture RGB and Depth Images
        image_path, depth_path = get_image_from_realsense(save_root='./sample')

        #tmp: timestamp
        topic = "1. get image"
        times[len(times)] = time.time()
        print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))
            
        with open(image_path, 'rb') as image_file:
            base64_image = base64.b64encode(image_file.read()).decode('utf-8')
        with open(depth_path, 'rb') as depth_file:
            base64_depth = base64.b64encode(depth_file.read()).decode('utf-8')        

        #tmp: timestamp
        topic = "2-1. load image"
        times[len(times)] = time.time()
        print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

        data = json.dumps({'image': base64_image, 'depth': base64_depth})
        response = requests.post(url_detection + 'upload', data=data, headers={'Content-Type': 'application/json'})
        print(response.json())

        #tmp: timestamp
        topic = "2-2. upload image and UOAIS"
        times[len(times)] = time.time()
        print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))

        # visulization
        image_vis = cv2.imread(image_path)
        img_h, img_w = image_vis.shape[:2]
        color = (0, 200, 250)
        cv2.putText(image_vis, "ROI", (roi_x1,roi_y1-5), cv2.FONT_HERSHEY_SIMPLEX, 1, color=color, thickness=2)
        cv2.rectangle(image_vis, (roi_x1,roi_y1), (roi_x2,roi_y2), color=color, thickness=3)

        ## CROP SAVE
        save_root = 'crop_image'
        os.makedirs(save_root, exist_ok=True)
        bboxes = response.json()['prediction']
        image = cv2.imread(image_path)
        
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
            response = requests.post(url_classification+'upload', data=data, headers={'Content-Type': 'application/json'})
            output = response.json()

            output_dict['bbox'].append([x1,y1,x2,y2])
            output_dict['prediction'].append(output['prediction'])
            output_dict['confidence'].append(output['confidence'])

            if output['prediction'] == 'OOD':
                OOD_OUTPUT = base64_image

            # visulization
            if run_visualization:
                color = (0,0,255) if output['prediction']=='OOD' else (255,255,255)
                cv2.rectangle(image_vis, (x1,y1), (x2,y2), color=color, thickness=2)
                cv2.putText(image_vis, output['prediction'], (x1+5,y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1, color=color, thickness=2)

            #tmp: timestamp
            topic = "2-{}. get CLS result".format(idx)
            times[len(times)] = time.time()
            print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))
        

        #tmp: timestamp
        topic = "3. visualization"
        times[len(times)] = time.time()
        print("{:.4f} {}".format(times[len(times)-1]-times[len(times)-2], topic))


        # Finetuning
        prediction = np.array(output_dict['prediction'])
        if 'OOD' in prediction:
            COUNT += 1

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
                break

            # Search
            data = json.dumps({'image': OOD_OUTPUT})
            response = requests.post(url_search + 'upload', data=data, headers={'Content-Type': 'application/json'})
            finetune_list = [response.json()['result']['top1']['data_id']]


            # Finetuning
            data = json.dumps({'data_list': finetune_list, 'strategy': 'classifier_tuning'})
            response = requests.post(url_classification + 'finetune', data=data, headers={'Content-Type': 'application/json'})
            print(response.json())


            # Reset COUNT
            COUNT = 0



        # print status
        (x1, y1), (x2, y2) = (img_w-250, 0), (img_w, 35)
        image_vis[y1:y2, x1:x2, :] = 0
        cv2.putText(image_vis, 'CLASSIFICATION', (x1+5,y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255,255,255), thickness=2)
        image_vis_res = cv2.resize(image_vis, (vis_w, vis_h))
        cv2.imshow("Visualization", image_vis_res)
        key = cv2.waitKey(1)
        if key == 27: # esc
            cv2.destroyAllWindows()
            break


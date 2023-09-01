import base64
import json
import requests
import os
import cv2


if __name__=='__main__':
    url_classification = 'http://localhost:8080/'  # Classification URL
    url_detection = 'http://localhost:8090/'  # Detection URL
    url_search = 'http://localhost:9000/'  # Search URL
    
    ## Load Detection
    data = json.dumps({'api_name': 'uoais_detection', 'device': 'cuda:0'})
    response = requests.post(url_detection + 'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
    
    ## Update Image
    image_path = './sample/image_3.png' # image_{1-4}.png
    depth_path = './sample/depth_3.png' # depth_{1-4}.png
    with open(image_path, 'rb') as image_file:
        base64_image = base64.b64encode(image_file.read()).decode('utf-8')

    with open(depth_path, 'rb') as depth_file:
        base64_depth = base64.b64encode(depth_file.read()).decode('utf-8')        
        
    data = json.dumps({'image': base64_image, 'depth': base64_depth})
    response = requests.post(url_detection + 'upload', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
    
    ## CROP SAVE
    save_root = 'crop_image'
    os.makedirs(save_root, exist_ok=True)
    bboxes = response.json()['prediction']
    image = cv2.imread(image_path)
    # image = cv2.resize(image, (W, H))
    for idx, bbox in enumerate(bboxes):
        x1, y1, x2, y2 = bbox
        cv2.rectangle(image, (x1,y1), (x2,y2), color=(255,255,255), thickness=2)
        image_crop = image.copy()[y1:y2, x1:x2]
        save_name = '{}_crop_{}.png'.format(os.path.basename(image_path), idx)
        save_file = os.path.join(save_root, save_name)
        cv2.imwrite(save_file, image_crop)
        print("... saved at", save_file) 
    
    save_file = os.path.join(save_root, os.path.basename(image_path))
    cv2.imwrite(save_file, image)
    print("... saved at", save_file) 
    
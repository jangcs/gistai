import base64
import json
import requests
import os
import cv2


if __name__=='__main__':
    url_classification = 'http://localhost:8080/'  # Classification URL
    url_detection = 'http://localhost:8090/'  # Detection URL
    url_search = 'http://localhost:9000/'  # Search URL
    
    ## Load Classification
    data = json.dumps({'api_name': 'food_classification', 'device': 'cuda:0'})
    response = requests.post(url_classification + 'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    

    ## Load Detection
    data = json.dumps({'api_name': 'uoais_detection', 'device': 'cuda:0'})
    response = requests.post(url_detection + 'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())


    ## Load Search
    data = json.dumps({'api_name': 'clip_search', 'device': 'cuda:0'})
    response = requests.post(url_search + 'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
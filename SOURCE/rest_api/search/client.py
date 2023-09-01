import base64
import json
import requests


if __name__=='__main__':
    url_search = 'http://localhost:9000/'  # Replace with your API url_classification
    
    ## Load Classification Model
    data = json.dumps({'api_name': 'clip_search', 'device': 'cuda:0'})
    response = requests.post(url_search+'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())


    ## PROTO Extraction
    data = json.dumps({'data_list': ['food_5/bulgogi', 'food_5/gimbap', 'food_5/kimchi', 
                                     'food_6/cola', 'food_6/orange', 'food_6/water',
                                     'food_7/cup', 'food_7/ladle', 'food_7/tongs']})
    response = requests.post(url_search + 'proto_extraction', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
        
    # ## Search
    # image_path = './sample/eggtart.png'
    # with open(image_path, 'rb') as image_file:
    #     base64_image = base64.b64encode(image_file.read()).decode('utf-8')
    # data = json.dumps({'image': base64_image})
    # response = requests.post(url_search + 'upload', data=data, headers={'Content-Type': 'application/json'})
    # print(response.json())

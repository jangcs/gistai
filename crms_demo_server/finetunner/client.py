import base64
import json
import requests


if __name__=='__main__':
    url_classification = 'http://localhost:8080/'  # Replace with your API url_classification
    
    
    ## Load Classification Model
    data = json.dumps({'api_name': 'bakery_classification', 'device': 'cuda:0'})
    response = requests.post(url_classification+'load_model', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
    
    ## Update Image
    # Classification
    image_path = './sample/eggtart.png'
    with open(image_path, 'rb') as image_file:
        base64_image = base64.b64encode(image_file.read()).decode('utf-8')
    data = json.dumps({'image': base64_image})
    response = requests.post(url_classification+'upload', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
    
    
    ## Finetuning
    finetune_list = ["food_2/egg_tart"]
    data = json.dumps({'data_list': finetune_list, 'strategy': 'classifier_tuning'})
    response = requests.post(url_classification+'finetune', data=data, headers={'Content-Type': 'application/json'})
    print(response.json())
# flask_app.py
import base64
import json
from flask import Flask, render_template, request, jsonify
from PIL import Image
from io import BytesIO
import os
import torch
import importlib
from copy import deepcopy
import shutil
from datetime import datetime

DEVICE = 'cpu'
MODEL_STORAGE = '/home/model_store'
MODEL = None
API_NAME = None

app = Flask(__name__)

def read_image(image_data):
    image = Image.open(BytesIO(base64.b64decode(image_data)))
    return image


@app.route('/')
def index():
    return render_template('index.html')


def load_model():
    global MODEL_STORAGE, DEVICE, MODEL, API_NAME
    
    # device = json.loads(request.data)['device']
    device = 'cuda:0'
    DEVICE = device
    
    # api_name = json.loads(request.data)['api_name']
    api_name = 'food_classification'

    spec = importlib.util.spec_from_file_location("module.name", os.path.join(MODEL_STORAGE, api_name, 'net.py'))
    foo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(foo)
    
    # Read Meta File
    with open(os.path.join(MODEL_STORAGE, api_name, 'meta.json'), 'r', encoding='utf-8') as f:
        meta = json.load(f)
    num_classes = len(meta['classes'])

    net = foo.load_pretrained_model(MODEL_STORAGE, api_name, num_classes)
    net = net.to(DEVICE)
    
    # Set Meta File
    net.meta = meta
    
    # Set Classes
    classes = [id.split('/')[-1] for id in meta['classes']]
    net.classes = classes
    
    # load transform
    transform = foo.load_transform()
    net.transform = transform
    
    MODEL = net
    API_NAME = api_name
    del net
    return {'status': 'success', 'api_name': api_name}


@app.route('/upload', methods=['POST'])
def upload():
    image_data = json.loads(request.data)['image']
    image = read_image(image_data)
    result = inference(image)
    return jsonify(result)

def inference(image):
    global MODEL, DEVICE
    MODEL.eval()
    
    if MODEL is None:
        return {'status': 'no model'}
    else:
        with torch.no_grad():
            confidence, pred = MODEL(image, check_ood=True, device=DEVICE)
        return {'status': 'success', 'prediction': pred, 'confidence': confidence.item()}



if __name__ == '__main__':
    res = load_model()
    print(res)
    app.run(host="0.0.0.0", port=8080)
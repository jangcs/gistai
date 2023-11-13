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
import numpy as np

DEVICE = 'cpu'
MODEL_STORAGE = '/home/search'
DATA_STORAGE = '/home/data_store'
MODEL = None
PROTO = None
CLASS_LIST = None
TRANSFORM = None
API_NAME = None

app = Flask(__name__)

def read_image(image_data):
    image = Image.open(BytesIO(base64.b64decode(image_data)))
    return image


@app.route('/')
def index():
    return render_template('index.html')


def load_model():
    global MODEL_STORAGE, DATA_STORAGE, DEVICE, MODEL, TRANSFORM, API_NAME
    
    # device = json.loads(request.data)['device']
    device = 'cuda:0'
    DEVICE = device
    
    # api_name = json.loads(request.data)['api_name']
    api_name = 'clip_search'

    spec = importlib.util.spec_from_file_location("module.name", os.path.join(MODEL_STORAGE, api_name, 'net.py'))
    foo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(foo)
    
    net, transform = foo.load_pretrained_model(MODEL_STORAGE, api_name, DEVICE)
    
    MODEL = net
    API_NAME = api_name
    TRANSFORM = transform
    del net, transform
    
    # Load Prototype
    update_proto()

    return {'status': 'success', 'api_name': api_name}

def update_proto():
    global CLASS_LIST, PROTO, DATA_STORAGE, DEVICE
    CLASS_LIST = []
    PROTO = []
    for dataset_name in os.listdir(DATA_STORAGE):
        dataset_path = os.path.join(DATA_STORAGE, dataset_name)
        for image_name in os.listdir(dataset_path):
            proto_path = os.path.join(DATA_STORAGE, dataset_name, image_name, 'prototype.npy')
            if os.path.isfile(proto_path):
                img_np_proto = np.load(proto_path)
                img_np_proto = img_np_proto.mean(0)
                img_np_proto /= np.linalg.norm(img_np_proto, axis=-1, keepdims=True)
                
                CLASS_LIST.append('%s/%s' %(dataset_name, image_name))
                PROTO.append(img_np_proto)
            else:
                continue
    
    PROTO = np.vstack(PROTO)
    PROTO = PROTO.astype(np.float16)
    PROTO = torch.from_numpy(PROTO).to(DEVICE)


@app.route('/upload', methods=['POST'])
def upload():
    image_data = json.loads(request.data)['image']
    image = read_image(image_data)
    result = inference(image)
    return jsonify(result)


def inference(image, topk=5):
    global MODEL, PROTO, CLASS_LIST, TRANSFORM, DEVICE
    
    if MODEL is None:
        return {'status': 'no model'}
    else:
        MODEL.eval()
        
        # Pre-process
        image = TRANSFORM(image)
        image = [image]
        image = torch.stack(image).to(DEVICE)
    
        # forward
        feature_images = MODEL.encode_image(image).to(DEVICE)
        feature_images_norm =  feature_images / feature_images.norm(dim=-1, keepdim=True)

        # calculate similarity with images
        similarity = (100.0 * feature_images_norm.float() @ PROTO.T.float())
        similarity = similarity[0]
        
        values, indices = similarity.topk(topk)
        output = {"top{}".format(idx+1):[] for idx in range(len(indices))}
        
        for i, (value, index) in enumerate(zip(values, indices)):
            output["top{}".format(i+1)] = {"data_id": CLASS_LIST[index], "similarity": value.item()}

        return {'status': 'success', 'result': output}



if __name__ == '__main__':
    res = load_model()
    print(res)
    app.run(host="0.0.0.0", port=9000)
    
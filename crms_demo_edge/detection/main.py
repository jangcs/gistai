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
# MODEL_STORAGE = '/home/model_store'
# MODEL_IMP_STORAGE = '/home/model_imp_store'
# DATA_STORAGE = '/home/data_store'


MODEL_STORAGE = '/home/model_store'
MODEL_IMP_STORAGE = '/home/model_imp_store'
DATA_STORAGE = '/home/data_store'

MODEL = None
TRANSFORM = None
API_NAME = None

app = Flask(__name__)

def read_image(image_data):
    image = Image.open(BytesIO(base64.b64decode(image_data)))
    return image


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/load_model', methods=['POST'])
def load_model():
    global MODEL_STORAGE, DEVICE, MODEL, TRANSFORM, API_NAME
    
    api_name = json.loads(request.data)['api_name']
    device = json.loads(request.data)['device']
    DEVICE = device
    
    spec = importlib.util.spec_from_file_location("module.name", os.path.join(MODEL_STORAGE, api_name, 'net.py'))
    foo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(foo)

    net = foo.load_pretrained_model(MODEL_STORAGE, api_name, DEVICE)
    

    # load transform
    W, H = net.cfg.INPUT.IMG_SIZE
    transform = foo.load_transform(W, H)
    
    MODEL = net
    TRANSFORM = transform    
    API_NAME = api_name
    
    del net
    return jsonify({'status': 'success', 'api_name': api_name})



@app.route('/upload', methods=['POST'])
def upload():
    image_data = json.loads(request.data)['image']
    depth_data = json.loads(request.data)['depth']
    
    image = np.array(read_image(image_data))
    depth = np.array(read_image(depth_data))
    
    result = inference(image, depth)
    return jsonify(result)



def inference(image, depth):
    global MODEL, DEVICE, TRANSFORM
    # MODEL.eval() # set eval mode at load_model()
    
    if MODEL is None:
        return {'status': 'no model'}
    else:
        # W, H = MODEL.cfg.INPUT.IMG_SIZE
        H, W = image.shape[:2]
        uoais_input = TRANSFORM(image, depth)        
        with torch.no_grad():
            outputs = MODEL(uoais_input)
        instances = MODEL.POST_PROC(outputs['instances'], H, W).to('cpu')
        bboxes = instances.pred_boxes.tensor.detach().cpu().numpy() 
        bboxes = np.int32(np.round(bboxes)).tolist()
        return {'status': 'success', 'prediction': bboxes}


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8090)
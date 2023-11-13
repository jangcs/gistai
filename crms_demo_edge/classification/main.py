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
MODEL_IMP_STORAGE = '/home/model_imp_store'
DATA_STORAGE = '/home/data_store'
MODEL = None
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
    global MODEL_STORAGE, DEVICE, MODEL, API_NAME
    
    device = json.loads(request.data)['device']
    DEVICE = device
    
    api_name = json.loads(request.data)['api_name']
    
    spec = importlib.util.spec_from_file_location("module.name", os.path.join(MODEL_STORAGE, api_name, 'net.py'))
    foo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(foo)
    
    net = foo.load_pretrained_model(MODEL_STORAGE, api_name)
    net = net.to(DEVICE)
    
    # Set Meta File
    with open(os.path.join(MODEL_STORAGE, api_name, 'meta.json'), 'r', encoding='utf-8') as f:
        meta = json.load(f)
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
    return jsonify({'status': 'success', 'api_name': api_name})


@app.route('/upload', methods=['POST'])
def upload():
    image_data = json.loads(request.data)['image']
    image = read_image(image_data)
    result = inference(image)
    return jsonify(result)


@app.route('/finetune', methods=['POST'])
def finetune():
    global MODEL, MODEL_STORAGE, MODEL_IMP_STORAGE, DATA_STORAGE, DEVICE, API_NAME
    old_list = MODEL.meta['classes']
    post_json = json.loads(request.data)
    
    new_list = post_json['data_list']
    strategy = post_json['strategy']
    
    class_list = old_list + new_list
    data_list = [os.path.join(DATA_STORAGE, data_ix) for data_ix in class_list]
    
    MODEL.finetune(data_list, strategy=strategy, epoch=10, lr=1e-3, batch_size=128, ratio=0.1, device=DEVICE)
    
    # Remove Old Finetune-file and Make New-folder
    new_folder = os.path.join(MODEL_IMP_STORAGE, API_NAME) 
    if os.path.isdir(new_folder):
        shutil.rmtree(new_folder)
    os.makedirs(new_folder, exist_ok=True)
    
    # Save Checkpoint
    checkpoint = deepcopy(MODEL.net.state_dict())
    torch.save(checkpoint, os.path.join(new_folder, 'model.pt'))
    
    # Save Meta file
    with open(os.path.join(MODEL_STORAGE, API_NAME, 'meta.json'), 'r', encoding='utf-8') as f:
        meta_imp = json.load(f)
    meta_imp['classes'] = class_list
    meta_imp['create_date'] = datetime.today().strftime('%Y-%m-%d')
    with open(os.path.join(new_folder, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta_imp, f)
        
    # Save net.py file
    old_net_path = os.path.join(MODEL_STORAGE, API_NAME, 'net.py')
    new_net_path = os.path.join(new_folder, 'net.py')
    shutil.copyfile(old_net_path, new_net_path)
    return {'status': 'success'}


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
    app.run(host="0.0.0.0", port=8080)
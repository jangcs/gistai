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
import crms

DEVICE = 'cpu'
MODEL_STORAGE = '/home/model_store'
MODEL_IMP_STORAGE = '/home/model_imp_store'
DATA_STORAGE = '/home/data_store'
MODEL = None
API_NAME = None

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/load_model', methods=['POST'])
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

    ## For CRMS Test....
    l = crms.crms_list()
    print(l)

    return {'status': 'success', 'api_name': api_name}


def replace_in_file(file_path, old_str, new_str):
    # 파일 읽어들이기
    fr = open(file_path, 'r')
    lines = fr.readlines()
    fr.close()
    
    # old_str -> new_str 치환
    fw = open(file_path, 'w')
    for line in lines:
        fw.write(line.replace(old_str, new_str))
    fw.close()


@app.route('/finetune', methods=['POST'])
def finetune():
    global MODEL, MODEL_STORAGE, MODEL_IMP_STORAGE, DATA_STORAGE, DEVICE, API_NAME
    old_list = MODEL.meta['classes']
    post_json = json.loads(request.data)
    
    new_list = post_json['data_list']
    strategy = post_json['strategy']
    
    class_list = old_list + new_list
    data_list = [os.path.join(DATA_STORAGE, data_ix) for data_ix in class_list]
    
    print("Finetunning...")
    print(data_list)
    MODEL.finetune(data_list, strategy=strategy, epoch=10, lr=1e-3, batch_size=128, ratio=0.1, device=DEVICE)
    # MODEL.finetune(data_list, strategy=strategy, epoch=5, lr=1e-3, batch_size=128, ratio=0.1, device=DEVICE)
    
    # # Remove Old Finetune-file and Make New-folder (UNNECESSARY in CRMS. Original-folder is used.)
    # new_folder = os.path.join(MODEL_IMP_STORAGE, API_NAME) 
    # if os.path.isdir(new_folder):
    #     shutil.rmtree(new_folder)
    # os.makedirs(new_folder, exist_ok=True)
    
    new_folder = os.path.join(MODEL_STORAGE, API_NAME)  # Original-folder is used for new_folder

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

    # # Save new net.py file 
    # replace_in_file( os.path.join(new_folder, 'net.py'), 'num_classes=4', 'num_classes=5' )
    # old_net_path = os.path.join(MODEL_STORAGE, API_NAME, 'net.py')
    # new_net_path = os.path.join(new_folder, 'net.py')
    # shutil.copyfile(old_net_path, new_net_path)

    os.chdir(new_folder)
    # crms.crms_add(['model.pt','meta.json', 'net.py'])
    crms.crms_add(['model.pt','meta.json'])
    # crms.crms_push(arg_version=datetime.today().strftime('%Y-%m-%d:%H.%M.%S'))
    crms.crms_push(arg_version='v'+datetime.today().strftime('%Y-%m-%d.%H.%M.%S'))

    return {'status': 'success'}

if __name__ == '__main__':
    res = load_model()
    print(res)
    # app.run(host="0.0.0.0", port=8070)
    app.run(port=80)

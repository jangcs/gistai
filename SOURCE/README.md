## Supported API List
- classification
    - bakery_classification
    - food_classification
    - tool_classification
    - drink_classification
- detection
- search


## Installation
- python3 설치
- 라이브러리 설치
  ```bash
      pip install opencv-python
      pip install pyrealsense2
  ```


## Structure
- 2023.08.30 ETRI 환경 셋팅에서는 ```$CLOUD = /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE```
- source code : ```$SRC = /home/robot/GIST_CLOUD_DEMO/SOURCE/```
  
```bash
# model_storage
- $CLOUD/model_store/
    - bakery_classification/
        - net.py                    # load network
        - meta.json                 # meta file
        - model.pt                  # pre-trained checkpoint
    - ...

# temporary model store (추가 학습 과정에서 발생되는 체크포인트 등을 저장하는 임시 저장소)
- $CLOUD/model_imp_store/
    - ...

# data_storage
- $CLOUD/data_store/
    - food_1/
        - croissant/
            - 1.png
            - 2.png
            - ...
        - pizza_bread/
            - ...
        - sandwich/
            - ...
        - sausage_bread/
            - ...
        - twisted_bread/
            - ...
    - food_2/
        - egg_tart/
            - ...
        - macaron/
            - ...
    - ...
```


## Usage
#### pull container from registry (처음 셋팅할때만 설정)
```bash
sudo docker pull hogili89/cloud:classification_ETRI
sudo docker pull hogili89/cloud:detection_ETRI
sudo docker pull hogili89/cloud:search_ETRI
```

#### run container
1. Run Classification
```bash
docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store:/home/model_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_imp_store:/home/model_imp_store \
            --net host --gpus all hogili89/cloud:classification_ETRI
```

2. Run Detection
```bash
docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store:/home/model_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_imp_store:/home/model_imp_store \
            --net host --gpus all hogili89/cloud:detection_ETRI
```

3. Run Search
```bash
docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store:/home/model_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_imp_store:/home/model_imp_store \
            --net host --gpus all hogili89/cloud:search_ETRI
```


#### client : send image and get results
```bash
python3 client_load_model.py              # Load Model
python3 client_camera.py                  # Image Capture and RestAPI call
```






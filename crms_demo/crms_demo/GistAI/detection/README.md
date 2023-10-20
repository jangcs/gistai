```
docker build --tag hogili89/cloud:detection_ETRI .

docker build --tag hogili89/cloud:detection_CRMS -f ./Dockerfile_crms .

#docker run -it -v /home/ailab-demo/Workspace/cloud_restAPI/data_store:/home/data_store \
#            -v /home/ailab-demo/Workspace/cloud_restAPI/model_store:/home/model_store \
#            -v /home/ailab-demo/Workspace/cloud_restAPI/model_imp_store:/home/model_imp_store \
#            --net host --gpus all hogili89/cloud:detection_ETRI

docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store:/home/model_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_imp_store:/home/model_imp_store \
            --net host --gpus all hogili89/cloud:detection_ETRI

docker run -it --net host --gpus all hogili89/cloud:detection_CRMS


```

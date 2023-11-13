```
docker build --tag hogili89/cloud:classification_ETRI .
#docker build --tag hogili89/cloud:classification_ETRI -f ./Dockerfile_crms .
docker build --tag hogili89/cloud:classification_CRMS -f ./Dockerfile_classification_crms .
docker build --tag hogili89/cloud:finetuner_CRMS -f ./Dockerfile_finetuner_crms .

docker image tag hogili89/cloud:classification_CRMS asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms


#docker run -it -v /home/ailab-demo/Workspace/cloud_restAPI/data_store:/home/data_store \
#            -v /home/ailab-demo/Workspace/cloud_restAPI/model_store:/home/model_store \
#            -v /home/ailab-demo/Workspace/cloud_restAPI/model_imp_store:/home/model_imp_store \
#            --net host --gpus all hogili89/cloud:classification_ETRI

docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store:/home/model_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_imp_store:/home/model_imp_store \
            --net host --gpus all hogili89/cloud:classification_ETRI

#docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store/food_classification:/home/model_store/food_classification \
#            --net host --gpus all hogili89/cloud:classification_CRMS
docker run -it -v /home/jangcs/crms_demo/food_classification:/home/model_store/food_classification \
            --net host --gpus all hogili89/cloud:classification_CRMS

docker run -it -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/data_store:/home/data_store \
            -v /home/robot/GIST_CLOUD_DEMO/CLOUD_STORE/model_store/food_classification:/home/model_store/food_classification \
            --net host --gpus all hogili89/cloud:finetuner_CRMS


```




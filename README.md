# Robot
## FoodListBuilder_node
```
cd ~/colcon_ws/src
docker build --tag foodlistbuilder:crms -f ./Dockerfile_FoodListBuilder .
docker image tag foodlistbuilder:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker run -it --net host --ipc host --name foodListBuilder asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
```

## FoodMention_node
```
cd ~/colcon_ws/src
docker build --tag foodmention:crms -f ./Dockerfile_FoodMention .
docker image tag foodmention:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker run -it --net host --ipc host --name foodMention asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
```

# Edge
## Classification
```
cd ~/crms_demo/classification
docker build --tag hogili89/cloud:classification_CRMS -f ./Dockerfile_classification_crms .
docker image tag hogili89/cloud:classification_CRMS asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
docker run -it -v /home/jangcs/crms_demo/food_classification:/home/model_store/food_classification \
            --net host --gpus all \
            asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
```
## detection
```
cd ~/crms_demo/classification
docker build --tag hogili89/cloud:detection_CRMS -f ./Dockerfile_crms .
docker image tag hogili89/cloud:detection_CRMS asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
docker run -it --net host --gpus all asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
```

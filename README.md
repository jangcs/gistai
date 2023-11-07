# Robot

## Xhost in the host
```
xhost +local:docker
```

## Camera_node (for IPC)
<!--
sudo su
source /opt/ros/foxy/setup.bash
source /home/jackal/colcon_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run crms_demo Camera_node
-->
```
cd ~/colcon_ws/src
docker build --tag camera:crms -f ./Dockerfile_Camera .
docker image tag camera:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/camera:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/camera:crms
docker run --privileged -it --net host --ipc host --name camera \
            -e DISPLAY=unix$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
            asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/camera:crms 
```

## FoodListBuilder_node
```
cd ~/colcon_ws/src
docker build --tag foodlistbuilder:crms -f ./Dockerfile_FoodListBuilder .
docker image tag foodlistbuilder:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker run -it --net host --ipc host --name foodListBuilder asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker run -it --net host --ipc host --name foodListBuilder \
            -e DISPLAY=unix$DISPLAY \
            --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
            asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms 
```


## FoodMention_node
```
cd ~/colcon_ws/src
docker build --tag foodmention:crms -f ./Dockerfile_FoodMention .
docker image tag foodmention:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker run -it --net host --ipc host --name foodMention \
            -e OPENAI_API_KEY=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 \
            asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
```

# Edge
## Classification_AI
```
cd ~/crms_demo/classification
docker build --tag hogili89/cloud:classification_CRMS -f ./Dockerfile_classification_crms .
docker image tag hogili89/cloud:classification_CRMS asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
docker run -it -v /home/jangcs/crms_demo/food_classification:/home/model_store/food_classification \
            --net host --gpus all \
            asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/classification:crms
```
## Detection_AI
```
cd ~/crms_demo/classification
docker build --tag hogili89/cloud:detection_CRMS -f ./Dockerfile_crms .
docker image tag hogili89/cloud:detection_CRMS asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
docker run -it --net host --gpus all asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/detection:crms
```

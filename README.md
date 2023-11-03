## FoodListBuilder_node
```
docker build --tag foodlistbuilder:crms -f ./Dockerfile_FoodListBuilder .
docker image tag foodlistbuilder:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
docker run -it --net host --name foorListBuilder asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodlistbuilder:crms
```

## FoodMention_node
```
docker build --tag foodmention:crms -f ./Dockerfile_FoodMention .
docker image tag foodmention:crms asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker push asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
docker run -it --net host --name foorMention asia-northeast3-docker.pkg.dev/cloudrobotai/crms-demo/foodmention:crms
```


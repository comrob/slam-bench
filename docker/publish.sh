export DOCKER_REGISTRY="ghcr.io/comrob"
export DOCKER_IMAGE="slam-bench"
export DOCKER_TAG="latest"
docker tag $DOCKER_IMAGE:$DOCKER_TAG $DOCKER_REGISTRY/$DOCKER_IMAGE:$DOCKER_TAG
docker push $DOCKER_REGISTRY/$DOCKER_IMAGE:$DOCKER_TAG
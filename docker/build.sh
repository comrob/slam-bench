docker compose -f docker-compose.yaml -f docker-compose-dev.yaml build
export DOCKER_REGISTRY="ghcr.io/sevahul"
export DOCKER_IMAGE="slam-bench"
export DOCKER_TAG="latest"
docker tag $DOCKER_IMAGE:$DOCKER_TAG $DOCKER_REGISTRY/$DOCKER_IMAGE:$DOCKER_TAG
default: build
build:
	docker build -t deployment_controller . --no-cache=true
run:
	docker stop controller || true
	docker rm controller || true
	docker run -it \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--env="XAUTHORITY=${XAUTH}" \
		--volume="${XAUTH}:${XAUTH}" \
		--privileged \
		--runtime=nvidia \
		--net=host \
		--name="controller" \
		deployment_controller bash
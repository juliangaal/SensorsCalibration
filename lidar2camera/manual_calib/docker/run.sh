#!/bin/bash
xhost + local:root

docker run --privileged -it --rm \
			--gpus all \
			-e DISPLAY=$DISPLAY \
			-v ${PWD}:/tmp/manual_calib \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev:/dev \
			$@

# xhost - local:root

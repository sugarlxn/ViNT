#!/bin/bash

sudo docker run --name vint2 --gpus all -it\
	--network=host \
	--runtime=nvidia \
	--privileged=true \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	--mount type=bind,source="/home/master/lxn/vint/vint_ws",target="/root/vint_ws" \
	nvcr.io/nvidia/cuda:12.1.0-runtime-ubuntu20.04 \
	bash

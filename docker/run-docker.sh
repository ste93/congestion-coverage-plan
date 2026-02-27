#!/bin/bash

# This script runs the image for the application.

xhost +;
docker run --rm -it --privileged --network host --pid host -e DISPLAY -v ./results:/app/congestion-coverage-plan/results -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 -e NVIDIA_DRIVER_CAPABILITIES=all ste93/congestion-coverage-plan:UC3_rebased_gem /bin/bash
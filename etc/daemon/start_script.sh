#!/bin/bash

CAM_MODULE_DIR="/home/shikoku-pc/cam_module/"

source /home/shikoku-pc/python/venv/bin/activate

cd "$CAM_MODULE_DIR" || exit 1

echo "current dir ($PWD)"

#exec ./cam_module > /dev/null 2>&1
exec ./cam_module


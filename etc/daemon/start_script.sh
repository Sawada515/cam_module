#!/bin/bash

CAM_MODULR_DIR="/home/shikoku-pc/cam_module

cd "$CAM_MODULE_DIR" || exit 1

echo "current dir $(PWD)"

exec ./cam_module

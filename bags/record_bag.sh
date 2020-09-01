#!/usr/bin/env bash
rosbag record --duration 60 -o ee6 \
            /scan \
            /tf \
            /usb_cam/image_raw \
            /usb_cam/camera_info \
            /odom \
            /wheel_odom

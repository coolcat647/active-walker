#!/usr/bin/env bash
rosbag record --duration 60 -o force_heavy \
            /scan \
            /tf \
            /usb_cam/image_raw \
            /usb_cam/camera_info \
            /odom \
            /wheel_odom \
            /force \
            /force_filtered \
            /wheel_odom

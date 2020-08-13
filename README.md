# active-walker
The repo is prepared for the active walker project for elders or people in mobility impaired.

## Prerequisite
a. Dependencies
```bash
$ sudo apt-get install -y libavcodec-dev \
  libswscale-dev \
  ros-melodic-driver-base \
  ros-melodic-vision-opencv \
  ros-melodic-pcl-ros \
  ros-melodic-camera-info-manager
```

## Setup 
a. Compile and build all packages
```bash
$ cd active-walker/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-6
```

___

## Reference
a. Motor controller: [FAULHABER MCDC3006S](https://www.faulhaber.com/fileadmin/Import/Media/EN_MCDC3006S_V2-5_DFF.pdf)

b. Servomotor: [FAULHABER 4490H048B](https://www.faulhaber.com/fileadmin/Import/Media/EN_4490_B_FMM.pdf)

c. Force sensor: [WACOH WEF-6A200-4-RCD](https://wacoh-tech.com/en/products/dynpick/200n_rcd.html)

d. Laser range sensor: [HOKUYO URG-04LX-UG01](https://www.hokuyo-aut.jp/search/single.php?serial=166)

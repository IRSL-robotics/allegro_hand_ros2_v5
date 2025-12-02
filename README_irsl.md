## (Optional) Docker installation

xhost +local:   # allow visualzation of container
docker pull osrf/ros:humble-desktop
docker run -it \
  --name allegro_ros2_gui \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e QT_X11_NO_MITSHM=1 \
  osrf/ros:humble-desktop \
  bash

## Setup notes
- Upgrade to the latest firmware of Allegro hand (tested on ver1.2)
- follow the following sections in README.md
  - Install the PCAN driver
  - Run main controller nodes
- run the following command:
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right|left TYPE:=B
- run keyboard node to check:
ros2 run allegro_hand_keyboards allegro_hand_keyboard


## example:
### run driver:
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=left TYPE:=B
### simple_allegro_example
ros2 run simple_allegro_example simple_control
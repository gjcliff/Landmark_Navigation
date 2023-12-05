# Semantic Mapping for Wheelchair Navigation
The goal of this project is to utilize a tablet to offer real-time feedback of the environment to the user. This feedback includes semantic labeling, which can be shown on the mapped environment when a landmark is identified through further inference. Additionally, for autonomous navigation towards a landmark, the user can initiate a plan by selecting it from the displayed landmarks. For more details, please go to my [portfolio post](https://r-shima.github.io/semantic_mapping.html).
## Hardware
* LUCI Wheelchair
* Intel RealSense D435i
* Windows Surface Pro 2 (with Ubuntu 22.04 LTS)
## Software
* ROS 2 Humble
* SLAM Toolbox
* robot_localization
* Nav2
* YOLOv8
* Foxglove Studio
## Important Note
I am not allowed to share any repository or code related to LUCI. If you have any questions, please contact the Assistive & Rehabilitation Robotics Laboratory (argallab) at Shirley Ryan AbilityLab. The subsequent steps are based on the assumption that you have access to their repositories.
## Cloning Packages
To clone the packages, use the following commands:
```
mkdir -p luci_ws/src
cd luci_ws/src
git clone https://github.com/r-shima/semantic_mapping.git
cd ..
vcs import < src/semantic_mapping/semantic_mapping.repos
```
## Set Up Docker
Once you have access to the Dockerfile for LUCI, build the image:
```
docker build -t argallab/luci:humble .
```
Then, run the container with the necessary mounts:
```
docker run -it --privileged \
-v /dev:/dev \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/user/luci_ws/src:/home/luci_ws/src \
-e DISPLAY \
-e QT_X11_NO_MITSHM=1 \
--name luci-humble \
--net=host argallab/luci:humble
```
## Building Packages
Start the container:
```
docker start -i luci-humble
```
Build packages:
```
cd home/luci_ws
colcon build
```
If there are missing dependencies, run the following for each missing dependency:
```
apt install ros-humble-<package_name>
```
## Set Up Surface Pro
Clone and build the packages:
```
mkdir -p luci_ws/src
cd luci_ws/src
git clone https://github.com/r-shima/semantic_mapping.git
cd ..
colcon build
```
Install Touchegg:
```
sudo apt install touchegg
```
Add the following in `~/.config/touchegg/touchegg.conf` to make a single tap emulate a left-click on Foxglove Studio:
```
<application name="Foxglove Studio">

    <gesture type="TAP" fingers="1" direction="">
        <action type="MOUSE_CLICK">BUTTON=1</action>
    </gesture>
    
</application>
```
## Quickstart
1. Go to your workspace in Docker (`home/luci_ws`) and run the following in different terminals to start LUCI, SLAM Toolbox, robot_localization, and Nav2:
   ```
   ros2 launch awl_launch awl_wheelchair.launch.py ip:=192.168.1.8
   ros2 launch awl_launch awl_teleop.launch.py joystick:=true ps3:=true
   ros2 param set /global_params assistance ROUTE
   ros2 launch awl_navigation luci_nav.launch.py slam_toolbox:=true
   ros2 launch awl_navigation navigation.launch.py
   ```
   Make sure to source the workspace by running `source install/setup.bash` before each command.
2. Connect to the RealSense camera via USB-C and start YOLOv8:
   ```
   ros2 launch object_detection yolov8_realsense.launch.py
   ```
3. Start the `landmark_manager` and `semantic_labeling` nodes:
   ```
   ros2 launch landmark_manager landmark_manager.launch.py
   ```
4. Launch the Foxglove bridge to connect ROS 2 to Foxglove Studio:
   ```
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```
5. On the Surface Pro, open Foxglove Studio. Go to "Open connection" and enter `ws://<computer_ip_address>:8765` for the WebSocket URL. Go to Layout -> Import from file and select `luci_map.json`, which is available in the foxglove directory of the landmark_manager package.
6. On the Surface Pro, start Touchegg:
   ```
   touchegg
   ```
7. On the Surface Pro, go to your workspace, source it, and launch the GUI:
   ```
   ros2 launch navigation_gui navigation_gui.launch.py
   ```
## Packages
* [landmark_manager](https://github.com/r-shima/semantic_mapping/tree/main/landmark_manager): This is a ROS 2 package that provides services for saving landmarks, canceling navigation, and navigating to landmarks. It also performs semantic labeling by publishing markers for detected doors and tables.
* [navigation_gui](https://github.com/r-shima/semantic_mapping/tree/main/navigation_gui): This is a ROS 2 package that contains a GUI that allows users to save landmarks and cancel navigation. The GUI displays buttons for detected doors and tables in a scroll area.
## Video

[Alt-Text](https://github.com/r-shima/semantic_mapping/assets/113070827/6bc5d5fd-c329-4380-9807-7c8d456aea30)
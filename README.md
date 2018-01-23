This is a ROS package that uses depth images to track hands. The code is based on the [realtime-tracker.cpp](https://github.com/IntelRealSense/hand_tracking_samples/blob/master/realtime-hand-tracker/realtime-tracker.cpp) of the [hand_tracking_samples](https://github.com/IntelRealSense/hand_tracking_samples) repository.

---

## Dependencies
1) **submodules** - This repo contains a submodule of a **[fork](https://github.com/ut-ims-robotics/hand_tracking_samples)** (has all unnecessary components removed) of the *hand_tracking_samples*. So be sure that you also clone the contents of the submodule:
```
git clone --recursive https://github.com/ut-ims-robotics/depth_cam_hand_tracker.git
```
2) **clang compiler** - The headers which are included from *hand_tracking_samples* were developed via clang compiler (and in c++14 standard):
```
 sudo apt install clang
```

3) **glfw3**
```
 sudo apt install libglfw3 
```

## Usage
**Tracker node:**
```
 rosrun depth_cam_hand_tracker tracker
```

 * **Subscribed topics (required)**

| Topic                       | Type
| ----------------------------|-------------
| *"camera/depth/camera_info"*| [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) 
| *"camera/depth/image"*      | [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

 * **Published topics**

| Topic                   | Type
| ------------------------|-------------
| *"hand_tracker_output"* | human_msgs/Hands

 * **Description**

Waits for depth camera parameters (camera_info), after which it starts to update the hand tracker based on incoming depth images. The tracker is far from perfect (as described in [hand_tracking_samples](https://github.com/IntelRealSense/hand_tracking_samples#known-limitations)). Please refer to the original description about the usage of the tracker (ignore the requirement for a RealSense camera): https://github.com/IntelRealSense/hand_tracking_samples/blob/master/realtime-hand-tracker/readme.md

## Future work
This repo is currently in a "proof-of-concept" state, which means that lots of things can be improved, such as:
 * Support depth images with other resolutions (currently 640x480)
 * Support other image encoding formats, namely 16UC1 (currently only 32FC1 is tested and supported)
 * Update the camera parameters dynamically and efficiently

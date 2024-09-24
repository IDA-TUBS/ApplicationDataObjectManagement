# ADOM - Application Data Object Management

## Build configuration
* **LOG=ON** enables logging statements
* **CONSOLE=ON** Adds a console log
* **FILE=ON** Adds a file log
* **EXAMPLE_WITHOUT_LOCALHOST=ON** enables ip addresses other than localhost

Usage: cmake .. -DLOG=ON -DCONSOLE=ON -DFILE=ON -DEXAMPLE_WITHOUT_LOCALHOST=ON in ApplicationDataObjectManagement/build/bin

## Build command for building the ADOM library
in ApplicationDataObjectManagement/build/bin:
```
sudo cmake --build .. --target install
```

# Build simple ROS2 example
Go to ROS2 example:
```
cd examples/ros2/simpleROS2Application/
```
Clean old build files:
```
rm -rf build install log
```
Build example with:
```
colcon build
```
Then setup with:
```
source install/setup.bash
```

## Execution of simple ros example
First make sure you download the provided rosbag and save it in ApplicationDataObjectManagement/examples/ros2/simpleROS2Application/rosbags.
Find the zipped rosbag here: https://cloud.tu-braunschweig.de/s/RYWdDpzXFTwkd3E
Then check the configuration of the ip addresses in ApplicationDataObjectManagement/includes/adom/paramaters.h.

in Terminal 1 at Host 1 (Publisher):
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_writer_adom.xml
```

in Terminal 2 at Host 2 (Subscriber):
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_reader_adom.xml
```
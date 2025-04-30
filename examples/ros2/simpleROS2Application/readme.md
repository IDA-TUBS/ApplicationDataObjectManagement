# Simple ROS 2 Application 

## Download test data
First make sure you download the provided rosbag and save it in *ApplicationDataObjectManagement/examples/ros2/simpleROS2Application/rosbags*.    
Find the zipped rosbag here: https://cloud.tu-braunschweig.de/s/RYWdDpzXFTwkd3E. 

## ROS 2
This is a test example for a ROS 2 application.
Make sure you have ros 2 humble installed. Find instructions on ROS 2 humble installation here: https://docs.ros.org/en/humble/Installation.html.

## Configure the library 
Then check the configuration of the ip addresses and topics in the json files in */ApplicationDataObjectManagement/examples/ros2/simpleROS2Application/config*.
Currently, the library only supports the simple configuration of a single publisher and subscriber, as is needed in this example.
Please make sure, that your configured topic number in *topics.json* is also defined in the Topic struct in */ApplicationDataObjectManagement/include/adom/parameters.h*

## Build simple ROS 2 example
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

## Execution of simple ROS 2 example
   
There are different launch files to launch different configurations of the simple example.  

### Execution of the Publisher and Subscriber Node on the same Host
This example only serves to easily demonstrate the operation of the library. Since utilizing this library brings its main benefits when communicating large data objects costs many communication resources, e.g. distributed publisher/ subscriber nodes that cannot communicate via shared memory, shared memory communication should be disabled for this example.

#### Launching the application with state-of-the-art DDS communication
The application with state-of-the-art DDS communication of the camera image sample can be launched with the following commands:

in a Terminal:
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_application_same_host.xml
```

#### Launching the application using ADOM library
The application using ADOM library to communicate the camera image sample can be launched with the following commands:

in a Terminal:
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_application_same_host_adom.xml
```

### Execution of the Publisher and Subscriber Node on different Hosts
Launching the simple ROS 2 example on different hosts demonstrates the benefits of the ADOM library.
Less packets will be send via the network and the relevant regions of interest of camera frames arrive faster.

#### Launching the application with state-of-the-art DDS communication
The application with state-of-the-art DDS communication of the camera image sample can be launched with the following commands:

in Terminal 1 at Host 1 (Publisher):
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_writer.xml
```

in Terminal 2 at Host 2 (Subscriber):
```
cd examples/ros2/simpleROS2Application/
source install/setup.bash
ros2 launch launcher simple_roi_reader.xml
```

#### Launching the application using ADOM library
The application using ADOM library to communicate the camera image sample can be launched with the following commands:

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
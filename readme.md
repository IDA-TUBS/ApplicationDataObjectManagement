# ADOM - Application Data Object Management

This repository contains a small, proof-of-concept library implementation for ROS 2/DDS compatible *Application Data Object Management*. The library itself also comes with a simple example: *simpleROS2Application*.
This example is an abstracted version of the traffic light detection from Autoware.universe, an open source software stack for autonomous driving. The example consists of two publishers and one subscriber. One of the publisher nodes distributes camera images. The other publisher publishes information about where traffic lights are located in these images. These camera images and regions of interest were previously recorded in the form of a rosbag during a test drive using the digital twin simulator AWSIM for the Autoware software stack. The ROS2 node *roi_reader* contains a subscriber that receives both inputs, i.e. camera images and regions of interest (ROI), and then marks the ROI with a green box. 

This test application is perfect for testing the function of the Application Data Object Management library, which allows only certain parts of an object to be transferred (like a region of interest) instead of an entire sample. 

For more information about how to work with and execute the ROS2 example, read the readme.md in *ApplicationDataObjectManagement/examples/ros2/simpleROS2Application/*





## Build configuration
* **LOG=ON** enables logging statements
* **CONSOLE=ON** Adds a console log
* **FILE=ON** Adds a file log

Usage: cmake .. -DLOG=ON -DCONSOLE=ON -DFILE=ON in *ApplicationDataObjectManagement/build/bin*

## Build command for building the ADOM library
in *ApplicationDataObjectManagement/build/bin*:
```
sudo cmake --build .. --target install
```


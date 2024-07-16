# OAK Driver
This package contains an ROS 1 driver for the Luxonis OAK cameras.

It contains an extended versions of the [depthai ros examples](https://github.com/luxonis/depthai-ros/tree/ros-release/depthai_examples) with more control options for the cameras. The additional features are:
 * Encoding of the video streams
 * Camera control via dynamic reconfigure
 * Support for multiple devices

Its currently tested with OAK-1-PoE / OAK-D-PoE / OAK-D-S2-PoE / OAK-1-Max cameras.

## Dependencies
 * depthai-core v2.17.4
 * depthai-ros 2.5.3

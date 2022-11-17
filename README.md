# OAK Driver
This package contains an ROS 1 driver for the Luxonis OAK cameras.

It contains extended versions of the [depthai ros examples](https://github.com/luxonis/depthai-ros/tree/ros-release/depthai_examples) with more control options for the cameras. The additional features are:
 * Encoding of the video streams
 * Camera control via dynamic reconfigure
 * Support for multiple devices

Its currently tested with OAK-1-PoE cameras and will also be extended for the OAK-D-PoE stereo cameras in the future.

## Dependencies
 * depthai-core v2.17.4
 * depthai-ros 2.5.3

# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

import av
from fractions import Fraction


class encoded_rgb_subscriber:

    def __init__(self):

        #ToDo get this information by the incomming stream!
        codec = "h264"
        fps = 15

        self._output_container = av.open('video.mp4', 'w')
        self._stream = self._output_container.add_stream(codec, rate=fps)
        self._stream.time_base = Fraction(1, 1000 * 1000) # Microseconds

        self._firstFrameTime = None

        if codec == "mjpeg":
            # We need to set pixel format for MJEPG, for H264/H265 it's yuv420p by default
            self._stream.pix_fmt = "yuvj420p"

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/oak_rgb_publisher/compressed", CompressedImage, self.callback,  queue_size = 5)


    def callback(self, msg):
        if not self._firstFrameTime :
            self._firstFrameTime = msg.header.stamp.to_time() * 1000 * 1000

        packet = av.Packet(msg.data)

        msgTime = msg.header.stamp.to_time() * 1000 * 1000
        packet.pts = int( msgTime - self._firstFrameTime)
        
        self._output_container.mux_one(packet) # Mux the Packet into container
        
    def __del__(self):
        self._output_container.close()
    
        

def main(args):
    '''Initializes and cleanup ros node'''
    enco = encoded_rgb_subscriber()
    rospy.init_node('encoded_rgb_subscriber')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

if __name__ == '__main__':
    main(sys.argv)

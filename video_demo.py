#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Quaternion,
)
from moveit_commander import conversions
import pos_tools
import conv_tools
import vis_tools
import math
import time
import multiprocessing
from baxter_core_msgs.msg import CollisionDetectionState
import goer
import watchdog
import baxter_tools

def main():

    rospy.init_node('arm')
    gripper = baxter_interface.GeckoGripper('left')
    
    while(True):
        print 'ready'
        key = raw_input()
        if key == 'x':
            rospy.signal_shutdown('Keyboard shutdown')
            return
        elif key == 'i':
            print baxter_tools.get_pose('left')
        elif key == 'g':
            gripper.command_position(gripper.primed_pos, block=True)
            baxter_tools.go_to('left', x=0.8, y=0.36, z=0.05, xr=180, yr=0, zr=180)
            gripper.pickup()
            baxter_tools.go_relative('left', dz=0.07)
            baxter_tools.go_to('left', x=0.58, y=0.1)
            baxter_tools.go_relative('left', dz=-0.08, timeout=1.5)
            gripper.command_position(gripper.released_pos, block=True)
            baxter_tools.go_relative('left', dz=0.1)
            gripper.command_position(gripper.primed_pos)
            
        elif key == ' ':
            gripper.command_position(50)
        elif key == 's':
            gripper.command_position(gripper.primed_pos)
            
if __name__ == '__main__':
    main()   
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    

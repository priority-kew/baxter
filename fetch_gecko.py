#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_tools
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

################################################################################
#   User-defined variables
################################################################################

# Arm (right or left) used to pick and place object
arm = 'left'

# Coordinates of gripper center
center_x = 525
center_y = 260

# Pixels object can be off center
pixel_tolerance = 30

# Arm speed
speed = 0.3

# Object to find
thing_path = '/home/baxter/ros_ws/src/baxter_tools/share/images/test.jpg'

################################################################################

class glass_fetcher:

    def __init__(self):
        # Physical interfaces
        self.limb = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.GeckoGripper(arm)
        self.head = baxter_interface.Head()

        # Virtual attributes
        self.angle_corrected = False
        self.finished = 3
        self.moving = False
        self.tol = pixel_tolerance
        self.prev_err = 0
        self.finder = baxter_tools.thing_finder(cv2.imread(thing_path, cv2.IMREAD_GRAYSCALE), min_match_count=8)

        # Gripper center coordinates
        self.gx = center_x
        self.gy = center_y

        # Set arm speed
        self.limb.set_joint_position_speed(speed)

#        baxter_tools.go_vertical(arm)
        baxter_tools.go_to(arm, x=0.75, y=0.36, z=0.05, xr=180, yr=0)

        # Get range to surface in meters
        self.dist = float(baxter_interface.analog_io.AnalogIO(arm + '_hand_range').state() / 1000.0)
        print 'dist = ', self.dist

        # Calibrate gripper
        if not self.gripper.calibrated():
            self.gripper.calibrate()

        # Prepare for image processing
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cameras/' + arm + '_hand_camera/image',Image,self.callback, queue_size=1)

    def callback(self, data):
        # Convert ROS image message to opencv format
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
#        cv2.imwrite('/home/baxter/ros_ws/src/baxter_tools/share/images/test.jpg', cv_img)
        # Find centroid of object
        [(cx,cy),approx,self.angle] = self.finder.find_thing(cv_img)
        
        if cx != -1:
            # Annotate camera image
            purple = [150, 0, 150]
            red = [0, 0, 255]
            blue = [255, 0, 0]
            cv2.drawContours(cv_img, approx, -1, purple, 8) 
            cv2.circle(cv_img, (cx, cy), 5, red, -1)
            cv2.circle(cv_img, (self.gx, self.gy), 5, blue, -8)

            # Move toward centering gripper over contour
            self.move_center(cx, cy)

        # Briefly display annotated camera image
        cv2.imshow("Object search", cv_img)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('x'):
            rospy.signal_shutdown('User shutdown')
        elif key == ord('n'):
            self.head.command_nod(0)
            time.sleep(0.5)
            self.finished = 0
            self.angle_corrected = False
            self.moving = False
        elif key == ord('d'):
            self.head.command_nod(0)
            time.sleep(0.5)
            self.head.command_nod(0)
            self.dist = float(baxter_interface.analog_io.AnalogIO(arm + '_hand_range').state() / 1000.0)
        

    def move_center(self, cx, cy):
        # If objective completed, return
        if self.finished >= 3:
            return

        # If arm still moving, do nothing
        try:
            if not baxter_tools.is_trans_still(arm):
                self.finished = 0
                return
        except KeyError:
            self.finished = 0
            return

        # Find pixel distance between object center and gripper center
        x_err = cx - self.gx
        y_err = cy - self.gy
        err = math.sqrt(x_err * x_err + y_err * y_err)
        print 'error =', err

        # Check whether pixel error is within tolerance
        if err < self.tol:
            self.finished += 1
            if self.finished == 3:
                if not self.angle_corrected:
                    if self.angle < 90.0:
                        ang = 180.0 - self.angle
                    else:
                        ang = 360.0 - self.angle
                    print 'angle =', ang
                    baxter_tools.go_to(arm, zr=ang)
                    self.angle_corrected = True
                    self.finished = 0
                    return
                

                try:
                    vert_off = self.gripper.pickup()
                except KeyError:
                    baxter_tools.set_z_distance(arm, 0.2)
                    self.finished = 0
                    self.angle_corrected = False
                    return
                baxter_tools.go_relative(arm, dz=0.05)
                '''
                if not self.gripper.gripping():
                    self.gripper.open()
                    pos_tools.command_shake()
                    self.finished = 0
                    return
                '''
                baxter_tools.go_to(arm, x=0.75, y=0.36)
                self.gripper.putdown()
                baxter_tools.set_z_distance(arm, self.dist)
            return
        else:
            self.finished = 0

        # If a motion command has been given but not carried out, do nothing
        if self.moving and abs(err - self.prev_err) < 20:
            return
        else:
            self.moving = False

        # Attempt IK to move arm into position
        try:
            pos_tools.move_point([cx,cy], [self.gx,self.gy], self.dist, arm)
            self.moving = True
            self.prev_err = err
        except:
            print 'ERROR - probably object found out of reach'

def camera_setup():
    # Open hand-camera
    try:
        baxter_interface.camera.CameraController('head_camera').close()
    except Exception:
        pass
    camera = baxter_interface.camera.CameraController(arm + '_hand_camera')
    camera.resolution = [960, 600]
    camera.gain = 45
    camera.open()

def main(args):
    rospy.init_node('glass_fetcher', anonymous=True)
    camera_setup()
    bf = glass_fetcher()
    print 'Press x to quit, n to (re)start or d to re-find table distance'
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

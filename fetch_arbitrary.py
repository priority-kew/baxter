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
import signal
import subprocess
import multiprocessing

################################################################################
#   User-defined variables
################################################################################

# Arm (right or left) used to pick and place object
arm = 'right'

# Coordinates of gripper center
center_x = 525
center_y = 260

# Pixels object can be off center
pixel_tolerance = 15

# Arm speed
speed = 0.3

# Black and white image of gripper to edit out of camera image
mask = '/home/baxter/ros_ws/src/baxter_tools/share/images/mask1.jpg'

# Object to find
thing_path = '/home/baxter/ros_ws/src/baxter_tools/share/images/gdbg.jpg'

################################################################################

class thing_fetcher:

    def __init__(self):
        # Physical interfaces
        self.limb = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.Gripper(arm)
        self.head = baxter_interface.Head()

        # Virtual attributes
        self.angle_corrected = False
        self.finished = 3
        self.moving = False
        self.tol = pixel_tolerance
        self.prev_err = 0
        self.thing = cv2.imread(thing_path, cv2.IMREAD_GRAYSCALE)

        # Gripper center coordinates
        self.gx = center_x
        self.gy = center_y

        # Get range to surface in meters
        self.dist = float(baxter_interface.analog_io.AnalogIO(arm + '_hand_range').state() / 1000.0)

        # Set arm speed
        self.limb.set_joint_position_speed(speed)

        # Calibrate gripper
        if not self.gripper.calibrated():
            self.gripper.calibrate()

        # Prepare for image processing
        self.bridge = CvBridge()
        self.gmask = cv2.imread(mask, cv2.IMREAD_GRAYSCALE)
        self.image_sub = rospy.Subscriber('/cameras/' + arm + '_hand_camera/image',Image,self.callback, queue_size=1)

    def callback(self, data):

        # Convert ROS image message to opencv format
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # Find centroid of largest dark object
        rvr_comm,sdr_comm = multiprocessing.Pipe(duplex=False)
        p = multiprocessing.Process(target=vis_tools.find_thing,args=(cv_img, self.thing,sdr_comm,))
        p.start()
        while p.is_alive():
            time.sleep(0.1)
        if p.exitcode == 0:
            [(cx, cy), approx, angle] = rvr_comm.recv()
            rvr_comm.close()
            if angle != 0.0:
                self.angle = angle
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
            if not pos_tools.is_trans_still(arm):
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
                    pos_tools.go_to_rotation(ang, arm)
                    self.angle_corrected = True
                    self.finished = 0
                    return


                try:
                    vert_off = pos_tools.set_z_distance(0.04, arm)
                except KeyError:
                    pos_tools.set_z_distance(0.2, arm)
                    self.finished = 0
                    self.angle_corrected = False
                    return
                self.gripper.close()
                pos_tools.go_vertical_relative(vert_off * -1, arm)
                if not self.gripper.gripping():
                    self.gripper.open()
                    pos_tools.command_shake()
                    self.finished = 0
                    return
                pos_tools.go_to(0.7, -0.35, 0.24, arm)
                home_off = self.dist - 0.13
                pos_tools.go_vertical_relative(-1 * home_off, arm)
                self.gripper.open()
                pos_tools.go_vertical_relative(home_off, arm)
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
    camera.gain = 20
    camera.open()

def main(args):
    rospy.init_node('thing_fetcher', anonymous=True)
    camera_setup()
    bf = thing_fetcher()
    print 'Press x to quit, n to (re)start or d to re-find table distance'

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

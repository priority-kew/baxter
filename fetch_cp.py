#!/usr/bin/env python

"""
This module runs a demonstration wherein a Baxter robot finds a given object
on a table and moves it to a preset location.
The object is given as an image on disk and recognized through the module
vis_tools.py using opencv. For the demo to be successful, the object must be
in the viewfield of whichever hand Baxter is using (it will not move the hand
to search).
Requires opencv 3.
"""

import sys
import rospy
import baxter_interface
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_srvs.srv
import pos_tools
import vis_tools
import math
import time

################################################################################
#   User-defined variables
################################################################################

# Arm (right or left) used to pick and place object
arm = 'right'

# Coordinates of gripper center in camera image
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

    """
    Take an image of a target object from file and, when directed to do so
    by user input, retrieve that object and return it to a home position.
    """

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
        self.finder = vis_tools.thing_finder(cv2.imread(thing_path,
                                                        cv2.IMREAD_GRAYSCALE))

        # Gripper center coordinates
        self.gx = center_x
        self.gy = center_y

        # Get range to surface in meters
        self.dist = float(baxter_interface.analog_io.AnalogIO(
            arm + '_hand_range').state() / 1000.0)

        # Set arm speed
        self.limb.set_joint_position_speed(speed)

        # Calibrate gripper
        if not self.gripper.calibrated():
            self.gripper.calibrate()

        # Prepare for image processing
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            '/cameras/' + arm + '_hand_camera/image', Image, self.callback,
            queue_size=1)

    def callback(self, data):

        """
        Called every time a new image is received from the camera.
        Find the target object in the image, mark it, and display
        the annotated image.
        Listen for user input.
        When directed, move to the target object, pick it up,
        and bring it back to a preset location.
        """

        # Convert ROS image message to opencv format
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # Find target object
        [(cx,cy),approx,self.angle] = self.finder.find_thing(cv_img)
        if cx != -1:
            # Annotate camera image
            purple = [150, 0, 150]
            red = [0, 0, 255]
            blue = [255, 0, 0]
            cv2.drawContours(cv_img, approx, -1, purple, 8)
            cv2.circle(cv_img, (cx, cy), 5, red, -1)
            cv2.circle(cv_img, (self.gx, self.gy), 5, blue, -8)

            # Over the course of multiple loops through,
            # move gripper to be centered over the object,
            # then pick up and move the object
            self.move_center(cx, cy)

        # Briefly display annotated camera image
        cv2.imshow("Object search", cv_img)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('x'):
            # Exit
            rospy.signal_shutdown('User shutdown')
        elif key == ord('n'):
            # Find and move object again
            self.head.command_nod(0)
            time.sleep(0.5)
            self.finished = 0
            self.angle_corrected = False
            self.moving = False
        elif key == ord('d'):
            # Refind distance to table
            self.head.command_nod(0)
            time.sleep(0.5)
            self.head.command_nod(0)
            self.dist = float(baxter_interface.analog_io.AnalogIO(
                arm + '_hand_range').state() / 1000.0)

    def move_center(self, cx, cy):

        """
        Given x and y coordinates from the camera image,
        move the arm to be centered over those coordinates,
        rotate the gripper to match the orientation of the object,
        pick the object up and deposit it at the home position.
        """

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
                # Rotate gripper to match object orientation
                if not self.angle_corrected:
                    if self.angle < 90.0:
                        ang = 180.0 - self.angle
                    else:
                        ang = 360.0 - self.angle
                    print 'angle =', ang
                    pos_tools.go_to_rotation(ang, arm)
                    self.angle_corrected = True
                    # Reset self.finished in case the rotation has caused
                    # the arm to move out of place
                    self.finished = 0
                    return

                # Pick up object
                try:
                    vert_off = pos_tools.set_z_distance(0.04, arm)
                except KeyError:
                    pos_tools.set_z_distance(0.2, arm)
                    self.finished = 0
                    self.angle_corrected = False
                    return
                self.gripper.close()
                pos_tools.go_vertical_relative(vert_off * -1, arm)
                # Check the gripper has actually acquired the object
                if not self.gripper.gripping():
                    self.gripper.open()
                    pos_tools.command_shake()
                    self.finished = 0
                    return
                # Put down the object
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
    """ Open hand-camera. """
    try:
        baxter_interface.camera.CameraController('head_camera').close()
    except Exception:
        pass
    camera = baxter_interface.camera.CameraController(arm + '_hand_camera')
    camera.resolution = [960, 600]
    camera.gain = 0
    camera.open()


def main(args):
    rospy.init_node('thing_fetcher', anonymous=True)
    camera_setup()
    bf = thing_fetcher()
    print 'Press x to quit, n to (re)start or d to re-find table distance'

    # Ensure rospy keeps running
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python

"""
This module runs a demonstration wherein a Baxter robot moves a pieces of glass
from a red square to a blue square using a gecko gripper.
The squares are found using opencv and tracked using an included class.
Both square must be visible in the robot's first view for the demo to complete
successfully.
"""

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
import time

################################################################################
#   User-defined variables
################################################################################

# Arm (right or left) used to pick and place object
arm = 'left'

# Coordinates of gripper center in camera image
center_x = 520
center_y = 290

# Arm speed
speed = 0.3

# Initial coordinates
init_x = 0.8
init_y = 0.2

################################################################################

class glass_mover:

    """
    Track a blue square and a red square in an image
    and move a piece of glass from red to blue when directed by user input.
    """

    def __init__(self):
        # Physical interfaces
        self.limb = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.GeckoGripper(arm)
        self.head = baxter_interface.Head()

        # Gripper center coordinates
        self.gx = center_x
        self.gy = center_y

        # Set arm speed
        self.limb.set_joint_position_speed(speed)

        self.gripper.command_position(self.gripper.primed_pos, block=True)
        pos_tools.set_z_distance(arm, 0.275)
        pos_tools.go_to(arm, x=init_x, y=init_y, xr=180, yr=0, zr=180)

        # Get range to surface in meters
        self.dist = float(baxter_interface.analog_io.AnalogIO(
            arm + '_hand_range').state() / 1000.0)

        # Prepare for image processing
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            '/cameras/' + arm + '_hand_camera/image', Image, self.callback,
            queue_size=1)

        self.blue_tracker = square_tracker('blue')
        self.red_tracker = square_tracker('red')
        self.dict_list = None

    def callback(self, data):

        """
        On each camera image, process for squares and display annotated image.
        Listen for key presses and if so directed, move a piece of glass
        from a red square to a blue square.
        """

        # Convert ROS image message to opencv format
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # Continually update square-tracking information
        squares = filter_squares(find_squares(self.img))
        self.dict_list = find_colored_squares(self.img, squares)
        self.blue_tracker.scrape(self.dict_list)
        self.red_tracker.scrape(self.dict_list)

        # Overlay colored outlines of squares on image
        for square_dict in self.dict_list:
            cv2.drawContours(self.img, [square_dict['contour']], -1,
                             square_dict['color'], 6 )

        # Display image with overlay
        cv2.imshow('squares', self.img)
        # Listen for key presses but keep updating
        key = cv2.waitKey(3) & 0xFF
        if key == ord('x'):
            # Exit
            rospy.signal_shutdown('User shutdown')
        elif key == ord('n'):
            # Run demo
            self.head.command_nod(0)
            time.sleep(0.5)
            self.blue_tracker.centered_pos = False
            self.red_tracker.centered_pos = False
            pos_tools.set_z_distance(arm, 0.275)

            # Go to blue square to memorize its position
            self.go_color('blue')
            # Go to red square
            self.go_color('red')
            # Pick up glass
            pos_tools.go_relative(arm, dz=-0.15)
            self.gripper.pickup()
            pos_tools.go_relative(arm, dz=0.15)
            # Go back to blue square and put down glass
            self.go_color('blue')
            pos_tools.go_relative(arm, dz=-0.1)
            self.gripper.putdown()
            time.sleep(1)
            pos_tools.go_relative(arm, dz=0.25)
            self.gripper.command_position(self.gripper.primed_pos)
        elif key == ord('d'):
            # Refind distance to table
            self.head.command_nod(0)
            time.sleep(0.5)
            self.head.command_nod(0)
            self.dist = float(baxter_interface.analog_io.AnalogIO(
                arm + '_hand_range').state() / 1000.0)

    def go_color(self, color):

        """
        Given a color ('red' or 'blue'), try to move to that color square.
        """

        if color == 'red':
            tracker = self.red_tracker
        if color == 'blue':
            tracker = self.blue_tracker

        # If centered position known, go there
        if tracker.centered_pos:
            pos = tracker.centered_pos
            pos_tools.go_to(arm, x=pos.x, y=pos.y)
            return

        # Otherwise go back to starting position (ideally last sighted position)
        pos = tracker.last_sighted_pos
        pos_tools.go_to(arm, x=init_x, y=init_y)

        coords = tracker.find_coordinates(self.dict_list)
        while not coords:
            coords = tracker.find_coordinates(self.dict_list)

        pos_tools.move_point(arm, tracker.find_coordinates(self.dict_list),
                                [self.gx,self.gy], self.dist, calib=0.0026)
        try:
            tracker.centered_pos = pos_tools.get_pose(arm)['position']
        except KeyError:
            print 'could not get position'
            tracker.centered_pos = False


class square_tracker:

    """
    Track either a red or a blue square.
    color - The color ('red' or 'blue') to be tracked.
    centered_pos - The position of the arm when centered over the target square
        (initially False).
    last_sighted_pos - The position of the arm at the second most recent
        sighting of the target.
    next_pos - The position of the arm at the most recent sighting
        of the target.
    last_sighted_coords - The coordinates of the target's center in the camera
        viewfield at the most recent sighting.
    """

    def __init__(self, color):
        self.color = color
        self.centered_pos = False
        self.last_sighted_pos = False
        self.next_pos = False
        self.last_sighted_coords = False

    def find_coordinates(self, dict_list):

        """
        Given a list of dictionaries containing square contours and
        their colors and centers, find the first square whose color matches
        the object's color and return its center. If none exist, return False.
        """

        for square_dict in dict_list:
            (b,g,r) = square_dict['color'][:3]
            if self.color == 'red':
                if r > b and r > g:
                    return square_dict['center']
            if self.color == 'blue':
                if b > r and b > g:
                    return square_dict['center']
        return False

    def scrape(self, dict_list):

        """
        Update information about the target square.
        If a centered_pos is already known, do nothing.
        Otherwise search the current camera image for the target square.
        If it is found, save the current position and the square's position
        in the viewfield.
        """

        if self.centered_pos:
            return
        coords = self.find_coordinates(dict_list)
        if coords:
            self.last_sighted_pos = self.next_pos
            self.next_pos = pos_tools.get_pose(arm)['position']
            self.last_sighted_coords = coords


def camera_setup():
    # Open hand-camera
    try:
        baxter_interface.camera.CameraController('head_camera').close()
    except Exception:
        pass
    camera = baxter_interface.camera.CameraController(arm + '_hand_camera')
    camera.resolution = [960, 600]
    camera.gain = 10
    camera.open()


def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )


def find_squares(img):

    """
    Given an opencv image, return a list of contours determined to be squares.
    """

    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)

            bin, contours, hierarchy = cv2.findContours(
                bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02 * cnt_len, True)
                if (len(cnt) == 4 and cv2.contourArea(cnt) > 1000
                        and cv2.contourArea(cnt)
                            < gray.shape[0] * gray.shape[1] * 0.9
                        and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos(cnt[i], cnt[(i+1) % 4],
                                     cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares


def filter_squares(squares):

    """
    Given a list of contours, return a list of the largest square contours
    with all contours inside them discarded.
    """

    # Sort for things that are actually squares, not just rectangles
    temp = []
    for square in squares:
        _,_,w,h = cv2.boundingRect(square)
        aspect_ratio = float(w)/h
        if abs(1 - aspect_ratio) < 0.1:
            temp.append(square)

    squares = temp
    temp = []
    clean_squares = []
    while len(squares) > 0:
        # Find largest remaining square
        max_area = 0
        for square in squares:
            area = cv2.contourArea(square)
            if area > max_area:
                max_area = area
                largest_square = square
        clean_squares.append(largest_square)

        # Discard all squares with centers inside largest square
        for square in squares:
            M = cv2.moments(square)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if cv2.pointPolygonTest(largest_square, (cx, cy), False) < 0:
                temp.append(square)
        squares = temp
        temp = []
    return clean_squares


def find_colored_squares(img, squares):

    """
    Given an opencv image and a list of contours, return a list of dictionaries
    each containing the contour, outline color and center point (as a tuple)
    of one of the contours.
    """

    dict_list = []
    for square in squares:
        # Find average color of contour outline
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros(grey.shape, np.uint8)
        cv2.drawContours(mask, [square], 0, 255, -1)
        kernel = np.ones((5,5), np.uint8)
        erosion = cv2.erode(mask, kernel, iterations=5)
        mask -= erosion
        color = cv2.mean(img, mask=mask)

        # Find center of contour
        M = cv2.moments(square)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        square_dict = {}
        square_dict['contour'] = square
        square_dict['color'] = color
        square_dict['center'] = (cx, cy)

        dict_list.append(square_dict)
    return dict_list


def main(args):
    rospy.init_node('glass_mover', anonymous=True)
    camera_setup()
    bf = glass_mover()
    print 'Press x to quit, n to (re)start or d to re-find table distance'

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

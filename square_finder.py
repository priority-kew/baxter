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
center_x = 520
center_y = 290

# Arm speed
speed = 0.3

# Initial coordinates
init_x = 0.8
init_y = 0.2

################################################################################

class glass_mover:

    def __init__(self):
        # Physical interfaces
        self.limb = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.GeckoGripper(arm)
        self.head = baxter_interface.Head()

        # Virtual attributes
        self.finished = 3

        # Gripper center coordinates
        self.gx = center_x
        self.gy = center_y

        # Set arm speed
        self.limb.set_joint_position_speed(speed)

        self.gripper.command_position(self.gripper.primed_pos, block=True)
        baxter_tools.set_z_distance(arm, 0.275)
        baxter_tools.go_to(arm, x=init_x, y=init_y, xr=180, yr=0, zr=180)

        # Get range to surface in meters
        self.dist = float(baxter_interface.analog_io.AnalogIO(arm + '_hand_range').state() / 1000.0)

        # Prepare for image processing
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cameras/' + arm + '_hand_camera/image',Image,self.callback, queue_size=1)

        self.blue_tracker = square_tracker('blue')
        self.red_tracker = square_tracker('red')
        self.dict_list = None


    def callback(self, data):
        # Convert ROS image message to opencv format
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        squares = filter_squares(find_squares(self.img))
        self.dict_list = find_colored_squares(self.img, squares)
        self.blue_tracker.scrape(self.dict_list)
        self.red_tracker.scrape(self.dict_list)
        
        for square_dict in self.dict_list:        
            cv2.drawContours( self.img, [square_dict['contour']], -1, square_dict['color'], 6 )

        for square in self.dict_list:
            if square['color'][0] > square['color'][1] and square['color'][0] > square['color'][2]:
                if self.finished == 0:
                    self.finished = 3
                    baxter_tools.move_point(arm, [square['center'][0], square['center'][1]], [self.gx,self.gy], self.dist, calib=0.0026)
                break
        cv2.imshow('squares', self.img)
        key = cv2.waitKey(3) & 0xFF
        if key == ord('x'):
            rospy.signal_shutdown('User shutdown')
        elif key == ord('n'):
            self.head.command_nod(0)
            time.sleep(0.5)
            self.blue_tracker.centered_pos = False
            self.red_tracker.centered_pos = False
            baxter_tools.set_z_distance(arm, 0.275)
            self.go_color('blue')
            self.go_color('red') 
            baxter_tools.go_relative(arm, dz=-0.15)           
            self.gripper.pickup()
            baxter_tools.go_relative(arm, dz=0.15)
            self.go_color('blue')
            baxter_tools.go_relative(arm, dz=-0.1)
            self.gripper.putdown()
            time.sleep(1)
            baxter_tools.go_relative(arm, dz=0.25)
            self.gripper.command_position(self.gripper.primed_pos)
        elif key == ord('d'):
            self.head.command_nod(0)
            time.sleep(0.5)
            self.head.command_nod(0)
            self.dist = float(baxter_interface.analog_io.AnalogIO(arm + '_hand_range').state() / 1000.0)    
        elif key == ord('s'):
            baxter_tools.go_relative(arm, dz=-0.15)  
        elif key == ord('a'):
            baxter_tools.go_relative(arm, dz=-0.05) 
        elif key == ord('w'):
            baxter_tools.go_relative(arm, dz=0.05)
        elif key == ord('g'):
            self.gripper.pickup()
        elif key == ord(' '):
            self.gripper.putdown()
        elif key == ord('b'):
            self.go_color('blue')
        elif key == ord('r'):
            self.go_color('red')

    def go_color(self, color):
        if color == 'red':
            tracker = self.red_tracker
        if color == 'blue': 
            tracker = self.blue_tracker

        if tracker.centered_pos:
            pos = tracker.centered_pos
            baxter_tools.go_to(arm, x=pos.x, y=pos.y)
            return

        pos = tracker.last_sighted_pos
#        baxter_tools.go_to(arm, x=pos.x, y=pos.y, z=pos.z)
        baxter_tools.go_to(arm, x=init_x, y=init_y)

        coords = tracker.find_coordinates(self.dict_list)
        while not coords:
            print 'caught'
            coords = tracker.find_coordinates(self.dict_list)

        baxter_tools.move_point(arm, tracker.find_coordinates(self.dict_list), [self.gx,self.gy], self.dist, calib=0.0026)
        try:
            tracker.centered_pos = baxter_tools.get_pose(arm)['position']
        except KeyError:
            print 'could not get position'
            tracker.centered_pos = False
        
class square_tracker:
    def __init__(self, color):
        self.color = color
        self.centered_pos = False
        self.last_sighted_pos = False
        self.next_pos = False
        self.last_sighted_coords = False

    def find_coordinates(self, dict_list):
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
        if self.centered_pos:
            return
        coords = self.find_coordinates(dict_list)
        if coords:
            self.last_sighted_pos = self.next_pos
            self.next_pos = baxter_tools.get_pose(arm)['position']
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
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            bin, contours, hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.contourArea(cnt) < gray.shape[0] * gray.shape[1] * 0.9 and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

def filter_squares(squares):
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
        max_area = 0
        for square in squares:
            area = cv2.contourArea(square)
            if area > max_area:
                max_area = area
                largest_square = square
        clean_squares.append(largest_square)
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
    dict_list = []
    for square in squares:
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros(grey.shape,np.uint8)
        cv2.drawContours(mask,[square],0,255,-1)

        kernel = np.ones((5,5),np.uint8)
        erosion = cv2.erode(mask, kernel, iterations=5)
        mask -= erosion
        color = cv2.mean(img, mask=mask)

        M = cv2.moments(square)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

#        _, _, angle = cv2.fitEllipse(square)
        
        square_dict = {}
        square_dict['contour'] = square
        square_dict['color'] = color
        square_dict['center'] = (cx, cy)
#        square_dict['angle'] = angle

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

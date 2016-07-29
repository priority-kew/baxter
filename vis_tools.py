#!/usr/bin/env python

"""
Library of functions for processing images,
nominally for Baxter but could be used elsewhere.
"""

import rospy
import baxter_interface
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
import multiprocessing
import time


def find_black_center(cv_img, msk):

    """
    Given an opencv image containing a dark object on a light background
    and a mask of objects to ignore (a gripper, for instance),
    return the coordinates of the centroid of the largest object
    (excluding those touching edges) and its simplified contour.
    If none detected or problem with centroid, return [(-1, -1), False].
    """

    # Convert to black and white
    (rows, cols, _) = cv_img.shape
    grey_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    grey_img = cv2.bilateralFilter(grey_img, 11, 17, 17)
    _, outlines = cv2.threshold(
        grey_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Subtract gripper
    msk_out = cv2.subtract(cv2.bitwise_not(outlines), msk)

    # Remove objects touching edges
    flood_fill_edges(msk_out, 30)

    # Find contours
    _, contours, _ = cv2.findContours(
        msk_out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return [(-1, -1), False]

    # Find largest contour
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            contour = cnt
            max_area = area

    # Approximate contour
    epsilon = 0.025 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Find centroid
    try:
        M = cv2.moments(approx)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return [(cx, cy), approx]
    except ZeroDivisionError:
        return [(-1, -1), False]


def flood_fill_edges(img, stride):

    """
    Check the strideth pixel along each edge of the image.
    If it is white, floodfill the image black from that point.
    """

    black = 0
    white = 255
    (rows, cols) = img.shape
    msk = np.zeros((rows+2, cols+2, 1), np.uint8)

    # Left and right edges
    i = 0
    while i < rows:
        if img[i, 0] == white:
            cv2.floodFill(img, msk, (0, i), black)
        if img[i, cols-1] == white:
            cv2.floodFill(img, msk, (cols-1, i), black)
        i += stride

    # Top and bottom edges
    i = 0
    while i < cols:
        if img[0, i] == white:
            cv2.floodFill(img, msk, (i, 0), black)
        if img[rows-1, i] == white:
            cv2.floodFill(img, msk, (i, rows-1), black)
        i += stride


def make_mask(limb, filename):

    """
    Given a limb (right or left) and a name to save to
    (in the baxter_tools/share/images/ directory),
    create a mask of any dark objects in the image from the camera
    and save it.
    """

    image_sub = rospy.Subscriber(
        '/cameras/' + limb + '_hand_camera/image',Image,callback)


    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError, e:
        print e

    msk = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY_INV)
    return msk


class thing_finder:

    """
    Class to recognize one object (given on instantiation) within different
    scenes. Communicates via pipe because opencv's flann.knnMatch has an
    unfortunate tendency to segfault.
    img must be under a certain size (somewhere around 4.8kB for jpg).
    """

    def __init__(self, img, min_match_count=10, flann_index=0, flann_trees=5,
            flann_checks=50):
        self.min_match_count = min_match_count
        self.thing = img
        self.rvr_comm,self.sdr_comm = multiprocessing.Pipe(duplex=False)

        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()

        # Find keypoints and descriptors of thing with SIFT
        self.keypoints, self.descriptors = self.sift.detectAndCompute(img,
                                                                      None)
        print 'num keypoints =', len(self.keypoints)

        # Initiate FLANN matcher
        index_params = dict(algorithm = flann_index, trees = flann_trees)
        search_params = dict(checks = flann_checks)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def find_thing(self, scene):

        """
        Search for thing within given scene.
        If it is found, returns a list containing the coordinates of its
        centroid within the scene, the contour surrounding it within the scene
        and its angle.
        If the object is not found or a segfault occurs, will instead return
        [(-1,-1),False,False].
        """

        # Run knn matching in a separate process to allow for segfaults
        p = multiprocessing.Process(target=self.sterile_match,args=(scene,))
        p.start()
        # Wait for process to finish or die
        while p.is_alive():
            time.sleep(0.01)

        if p.exitcode != 0:
            # Segfault
            return [(-1,-1),False,False]

        return self.rvr_comm.recv()

    def sterile_match(self, scene):
        # Find keypoints and descriptors in scene with SIFT
        scene_keypoints, scene_descriptors = self.sift.detectAndCompute(scene,
                                                                        None)
        # Match descriptors with FLANN
        matches = self.flann.knnMatch(self.descriptors,scene_descriptors,k=2)

        # Store all the good matches as per Lowe's ratio test.
        good = []

        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        print 'len(good) =', len(good)
        if len(good)<=self.min_match_count:
            # Thing not found
            self.sdr_comm.send([(-1,-1),False,False])
            return

        src_pts = np.float32(
            [self.keypoints[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        dst_pts = np.float32(
            [scene_keypoints[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        matchesMask = mask.ravel().tolist()

        h,w = self.thing.shape
        pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        # Contour
        cnt = np.int32(dst)

        # Find centroid
        try:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        except ZeroDivisionError:
            self.sdr_comm.send([(-1,-1),False,False])
            return

        # Find angle
        mid = [[[(cnt[0][0][0] + cnt[1][0][0])/2,
            (cnt[0][0][1] + cnt[1][0][1])/2]]]
        cnt = np.append(cnt, mid, axis=0)

        mid = [[[(cnt[1][0][0] + cnt[2][0][0])/2,
            (cnt[1][0][1] + cnt[2][0][1])/2]]]
        cnt = np.append(cnt, mid, axis=0)

        mid = [[[(cnt[2][0][0] + cnt[3][0][0])/2,
            (cnt[2][0][1] + cnt[3][0][1])/2]]]
        cnt = np.append(cnt, mid, axis=0)

        mid = [[[(cnt[3][0][0] + cnt[0][0][0])/2,
            (cnt[3][0][1] + cnt[0][0][1])/2]]]
        cnt = np.append(cnt, mid, axis=0)

        _, _, angle = cv2.fitEllipse(cnt)

        self.sdr_comm.send([(cx, cy), cnt, angle])








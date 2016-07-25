#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Quaternion,
)
import conv_tools
from moveit_commander import conversions
import math
import numpy as np
import multiprocessing
import time
import tf
import baxter_tools

# Informational

def get_pose(limb):

    """
    Return the endpoint pose of the given limb (right or left)
    as a dictionary.
    """

    if limb == 'right':
        return baxter_interface.limb.Limb('right').endpoint_pose()
    elif limb == 'left':
        return baxter_interface.limb.Limb('left').endpoint_pose()
    else:
        raise Exception("Expected argument 'right' or 'left' to get_posn.")

def is_trans_still(limb):

    """ 
    Return True if the magnitude of the linear velocity of the limb (right
    or left) is under 0.0001; otherwise return False.
    """
    
    vel_vec = baxter_interface.limb.Limb(limb).endpoint_velocity()['linear']
    vel_mag = math.sqrt(vel_vec.x * vel_vec.x + 
                        vel_vec.y * vel_vec.y + 
                        vel_vec.z * vel_vec.z)
    if vel_mag > 0.015:
        return False
    else:
        return True

# Motional
def go_to_pose(lmb, pose_msg, timeout=15.0):

    """ 
    Given a limb (right or left) and a desired pose as a stamped message,
    run inverse kinematics to attempt to find a joint configuration to yield
    the pose and then move limb to the configuration.
    After 5 seconds of attempts to solve the IK, raise an exception.
    """

    node = "ExternalTools/" + lmb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(pose_msg)

    # Allow 5 seconds to find an IK solution
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid
    if ik_response.isValid[0]:
        # convert response to joint position control dictionary
        limb_joints = dict(zip(ik_response.joints[0].name, 
                               ik_response.joints[0].position))
        # send limb to joint positions
        baxter_interface.limb.Limb(lmb).move_to_joint_positions(limb_joints, timeout=timeout)
    else:
        raise Exception("Failed to find valid joint configuration.")

def update_pose(pos, dx=0.0, dy=0.0, dz=0.0, dxr=0.0, dyr=0.0, dzr=0.0):

    """
    Update given pose by adding any supplied offsets:
    dx, dy, dz for translation;
    dxr, dyr, dzr for rotation.
    """

    new_posn = Point(pos.position.x + dx,
                     pos.position.y + dy,
                     pos.position.z + dz)
    elr = tf.transformations.euler_from_quaternion([pos.orientation.x, 
                                                    pos.orientation.y,  
                                                    pos.orientation.z,  
                                                    pos.orientation.w])
    new_elr = Point(elr[0] + dxr, elr[1] + dyr, elr[2] + dzr)
    qtn = Quaternion()
    qtn.x, qtn.y, qtn.z, qtn.w = tf.transformations.quaternion_from_euler(new_elr.x, new_elr.y, new_elr.z)
    pos.position = new_posn
    pos.orientation = qtn

def go_relative(lmb, dx=0.0, dy=0.0, dz=0.0, dxr=0.0, dyr=0.0, dzr=0.0, timeout=15.0):

    """
    Move the given limb by any supplied offsets:
    dx, dy, dz for translation;
    dxr, dyr, dzr for rotation (degrees).
    Set timeout (seconds) to interrupt the attempt early.
    """

    pos = conv_tools.dict_to_msg(get_pose(lmb))
    update_pose(pos, dx, dy, dz, dxr/180.0*math.pi, dyr/180.0*math.pi, dzr/180.0*math.pi)
    new_pose = baxter_tools.stamp(pos)
    go_to_pose(lmb, new_pose, timeout)

def go_to(lmb, x=np.nan, y=np.nan, z=np.nan, xr=np.nan, yr=np.nan, zr=np.nan, timeout=15.0):

    """ 
    Move the given limb to any supplied coordinates and rotations:
    dx, dy, dz for location;
    dxr, dyr, dzr for rotation (degrees).
    Keep all unsupplied parameters the same.
    """

    pos = conv_tools.dict_to_msg(get_pose(lmb))
    elr = tf.transformations.euler_from_quaternion([pos.orientation.x, 
                                                    pos.orientation.y,  
                                                    pos.orientation.z,  
                                                    pos.orientation.w])

    if np.isnan(x):
        x = pos.position.x
    if np.isnan(y):
        y = pos.position.y
    if np.isnan(z):
        z = pos.position.z

    if np.isnan(xr):
        xr = elr[0]
    else:
        xr = xr/180.0*math.pi

    if np.isnan(yr):
        yr = elr[1]
    else:
        yr = yr/180.0*math.pi

    if np.isnan(zr):
        zr = elr[2]
    else:
        zr = zr/180.0*math.pi

    new_posn = Point(x, y, z)
    qtn = Quaternion()
    qtn.x, qtn.y, qtn.z, qtn.w = tf.transformations.quaternion_from_euler(xr, yr, zr)

    pos.position = new_posn
    pos.orientation = qtn

    new_pose = conv_tools.stamp(pos)
    go_to_pose(lmb, new_pose, timeout)

def go_vertical(lmb):

    """
    Rotate the given limb to vertical, keeping all other pose elements the same.
    """

    go_to(lmb, xr=180.0, yr=0.0)

# Head
def command_shake():
    """Shake head once."""
    head = baxter_interface.Head()
    c = 70
    head.set_pan(0.2, speed=c)
    head.set_pan(-0.3, speed=c)
    head.set_pan(0.0, speed=c)

# Complex control
def move_point(lmb, dest, pnt, dist, calib=0.0021):

    """
    Given a point in an image, the desired location of that point, the
    distance of the camera from the surface, and a limb (right or left), 
    attempt to move the limb such that the point moves to the desired location.
    """

    # experimental meters per pixel at 1 meter
    cam_calib = calib

    pixel_dx = dest[0] - pnt[0]
    pixel_dy = dest[1] - pnt[1]
    x_offset = - pixel_dy * cam_calib * dist
    y_offset = - pixel_dx * cam_calib * dist

    go_relative(lmb, dx=x_offset, dy=y_offset)

def set_z_distance(lmb, goal, tol=0.005):

    """
    Assuming the arm is oriented vertically, use range-finder and IK control
    to set the distance of the range-finder from the nearest object.
    Return the distance moved.
    """

    # Distance to nearest object in meters
    dist = float(baxter_interface.analog_io.AnalogIO(lmb + '_hand_range').state() / 1000.0)

    error = goal - dist
    if abs(error) < tol:
        return error
    go_relative(lmb, dz=error)
    return error + set_z_distance(lmb, goal, tol)

def go_F_ctrl_vertical_relative(z_offset, F_max, lmb, speed=0.15):

    """
    Attempt to move the arm to z_offset unless F_max 
    (vertical force the limb is exerting) is reached first. 
    Return True if F_max is reached before z_offset; False otherwise.
    """

    arm = baxter_interface.limb.Limb(lmb)
    arm.set_joint_position_speed(speed)
    g = multiprocessing.Process(target=goer, args=(z_offset,lmb,))
    wd = multiprocessing.Process(target=watchdog, args=(F_max,lmb,))

    g.start()
    wd.start()

    while wd.is_alive():
        time.sleep(0.1)
    g.terminate()

def watchdog(F_max, lmb):
    rospy.signal_shutdown('cleaning')
    rospy.init_node('watchdog')
    while True:
        try:
            Fz = baxter_interface.limb.Limb(lmb).endpoint_effort()['force'].z
            print Fz
            if abs(Fz) > F_max:
                rospy.signal_shutdown('Fmax exceeded')
        except KeyError:
            pass

def goer(z_offset, lmb):
    rospy.signal_shutdown('cleaning')
    rospy.init_node('goer')
    go_relative(lmb, dz=z_offset)
    rospy.signal_shutdown('done moving')



    















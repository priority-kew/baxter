#!/usr/bin/env python

import rospy
from moveit_commander import MoveItCommanderException
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import tf

def dict_to_list(pose_dict):

    """
    Convert a pose dictionary to a list in the order x, y, z,
    orientation x, orientation y, orientation z, orientation w.
    """

    lst = []
    lst.append(pose_dict['position'].x)
    lst.append(pose_dict['position'].y)
    lst.append(pose_dict['position'].z)
    lst.append(pose_dict['orientation'].x)
    lst.append(pose_dict['orientation'].y)
    lst.append(pose_dict['orientation'].z)
    lst.append(pose_dict['orientation'].w)
    return lst

def dict_to_list_euler(pose_dict):

    """
    Convert a pose dictionary to a list in the order x, y, z,
    orientation x, orientation y, orientation z.
    """

    qtn = Quaternion()
    qtn.x = pose_dict['orientation'].x
    qtn.y = pose_dict['orientation'].y
    qtn.z = pose_dict['orientation'].z
    qtn.w = pose_dict['orientation'].w
    elr_x, elr_y, elr_z, = tf.transformations.euler_from_quaternion([qtn.x,qtn.y,qtn.z,qtn.w])
                                                   
    lst = []
    lst.append(pose_dict['position'].x)
    lst.append(pose_dict['position'].y)
    lst.append(pose_dict['position'].z)
    lst.append(elr_x)
    lst.append(elr_y)
    lst.append(elr_z)
    return lst

def list_to_dict(lst):

    """
    Convert a list to a pose dictionary, assuming it is in the order
    x, y, z, orientation x, orientation y, orientation z[, orientation w].
    """

    if len(lst) == 6:
        qtn = tf.transformations.quaternion_from_euler(lst[3], lst[4], lst[5])
    elif len(lst) == 7:
        qtn = Quaternion()
        qtn.x = lst[3]
        qtn.y = lst[4]
        qtn.z = lst[5]
        qtn.w = lst[6]
    else:
        raise MoveItCommanderException("""Expected either 6 or 7 elements 
                in list: (x,y,z,r,p,y) or (x,y,z,qx,qy,qz,qw)""")

    pnt = Point()
    pnt.x = lst[0]
    pnt.y = lst[1]
    pnt.z = lst[2]

    pose_dict = {
        'position': pnt,
        'orientation': qtn
    }
    return pose_dict   

def dict_to_msg(pose_dict):
    pose_msg = Pose()
    pose_msg.position = pose_dict['position']
    pose_msg.orientation = pose_dict['orientation']
    return pose_msg

def msg_to_dict(pose_msg):
    pose_dict = {
        'position': pose_msg.position,
        'orientation': pose_msg.orientation
    }
    return pose_dict

def stamp(pose_msg):

    """
    Convert an unstamped pose message to a stamped pose message
    in the frame 'base' with the current rospy time.
    """

    pose_msg_stamped = PoseStamped()
    pose_msg_stamped.pose = pose_msg
    pose_msg_stamped.header.frame_id = 'base'
    pose_msg_stamped.header.stamp = rospy.Time.now()
    return pose_msg_stamped

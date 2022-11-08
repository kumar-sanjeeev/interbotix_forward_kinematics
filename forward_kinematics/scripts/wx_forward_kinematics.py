import numpy as np
from math import pi, cos, sin, atan2, sqrt

def cal_transformation(theta, d, alpha, a):
    """
    Function to calculate the transformation matrix from the DH parameters

    Parameters:
    ----
    DH parameters: theta, d, alpha, a

    Returns:
    ----
    4x4 transformation matrix

    """
    transformation = np.array([[np.cos(theta), -np.sin(theta)* np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                                   [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                                   [0, np.sin(alpha), np.cos(alpha), d],
                                   [0,0,0,1]])
    
    return transformation

def forward_kinematics(joints):

    """
    Funtion to calculate the forward kinematics of the serial manipulator using the DH conventions

    Parameters:
    ----
    list of joint angles

    Returns:
    ----
    pose and orientation of the end-effector
    [x, y, z, roll, pitch, yaw]

    """

    theta_dash = np.arctan(3) 

    joint1 = joints[0] + np.pi
    joint2 = joints[1] + np.pi -theta_dash
    joint3 = joints[2] - theta_dash

    # DH parameters of the links 
    joints = [joint1, joint2, joint3]
    d = [0.1039, 0, 0]
    a = [0, .1581, .15]
    alpha = [np.pi/2, np.pi, np.pi/2]

    result = np.identity(4)

    for i in range(0,len(joints)):

        result =np.dot(result, cal_transformation(joints[i],d[i],alpha[i],a[i]))

    # position of the end-effector        
    x = result[0][3]
    y = result[1][3]
    z = result[2][3]

    # orientation of the end-effector----> in radian
    roll = np.arctan2(result[2,1],result[2,2])
    pitch =np.arctan2(-result[2,0],sqrt(result[2,1]*result[2,1] + result[2,2]*result[2,2]))
    yaw= np.arctan2(result[1,0],result[0,0])

    return [x, y, z, roll, pitch, yaw]

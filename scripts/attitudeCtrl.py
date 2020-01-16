#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
from nav_msgs.msg import Odometry
from simulator.msg import Torque, RPYAngles
from geometry_msgs.msg import Quaternion
import tf
import numpy as np
import quadrotorClass
from dynamic_reconfigure.server import Server
from simulator.cfg import attitudeCtrlCFGConfig


# node init
rospy.init_node('attitudeCtrl', anonymous=False)


# publishers
# -----------
# Torque
pubTorque = rospy.Publisher('torque', Torque, queue_size=50)


# control frequency
freq = 100.
Ts = 1./freq
rate = rospy.Rate(freq)

# quadrotor
quadrotor = quadrotorClass.Quadrotor()
# state components
yaw = 0.
pitch = 0.
roll = 0.
angularVel = np.zeros((3,1))
# angular ref
yawRef = 0.
pitchRef = 0.
rollRef = 0.
angularVelRef = np.zeros((3,1)) # zero for attitude stabilization


''' dynamic parameters '''
'''
# PID gains
kp = 0.5
ki = 0.
kd = 0.1
'''

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------
    global kp, ki, kd, kpYaw, kiYaw, kdYaw

    kp = float("""{Kp}""".format(**config))
    ki = float("""{Ki}""".format(**config))
    kd = float("""{Kd}""".format(**config))

    kpYaw = float("""{KpYaw}""".format(**config))
    kiYaw = float("""{KiYaw}""".format(**config))
    kdYaw = float("""{KdYaw}""".format(**config))

    return config
# -----------------------------------------------------------------------------


# server for dyamic parameters
srv = Server(attitudeCtrlCFGConfig, callbackDynParam)

# init dynamic parameters
kp = rospy.get_param('/attitudeCtrl/Kp', 52.2)
ki = rospy.get_param('/attitudeCtrl/Ki', 0.)
kd = rospy.get_param('/attitudeCtrl/Kd', 55.2)

kpYaw = rospy.get_param('/attitudeCtrl/KpYaw', 5.)
kiYaw = rospy.get_param('/attitudeCtrl/KiYaw', 0.)
kdYaw = rospy.get_param('/attitudeCtrl/KdYaw', 8.)


# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackRPYAnglesRef(data):
# -----------------------------------------------------------------------------
    global rollRef, pitchRef, yawRef
    
    #read yaw reference from message
    rollRef = data.roll
    pitchRef= data.pitch    
    yawRef = data.yaw
# -----------------------------------------------------------------------------        

# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global roll, pitch, yaw, angularVel    
    
    # read attitude angles from odom msg    
    quat = Quaternion()
    quat.x = data.pose.pose.orientation.x
    quat.y = data.pose.pose.orientation.y
    quat.z = data.pose.pose.orientation.z
    quat.w = data.pose.pose.orientation.w
    angles = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='sxyz')
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]
    # read angular speed from odom msg
    angularVel[0] = data.twist.twist.angular.x
    angularVel[1] = data.twist.twist.angular.y
    angularVel[2] = data.twist.twist.angular.z
# -----------------------------------------------------------------------------        
        
   
# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("RPYAnglesRef", RPYAngles, callBackRPYAnglesRef)


# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    
    #global quadrotor

    # init messages
    torqueMsg = Torque()
    
    # main loop
    while not rospy.is_shutdown():

        # attitude and reference        
        eta = np.array([[roll],[pitch],[yaw]])
        etaRef = np.array([[rollRef],[pitchRef],[yawRef]])

        # PID for attitude control
        u = -kp*(eta - etaRef) -kd*(angularVel - angularVelRef)

        OmegaxJOmega = np.cross(angularVel.T, np.dot(quadrotor.J, angularVel).T).T
        
        # yaw        
        yawErr = yawRef - yaw
        # TO DO: COMPLETE MODULOSASSESSMENT OF THE GROUND RISK IN THE OPERATION OF SMALL UNMANNED AERIAL VEHICLES 
        if (yawErr>np.pi):
            yawErr = yawErr - 2.*np.pi
        elif (yawErr<= -np.pi):
            yawErr = yawErr + 2.*np.pi
        yawErr = -yawErr
        
        uz = -kpYaw*(yawErr) -kdYaw*(angularVel[2] - angularVelRef[2])
        u[2] = uz          
        
        torque = OmegaxJOmega + np.dot(quadrotor.J, u)
        
        # msgs update        
        timeNow = rospy.Time.now()
        torqueMsg.header.seq += 1
        torqueMsg.header.stamp = timeNow     
        torqueMsg.torque.x = torque[0]        
        torqueMsg.torque.y = torque[1]
        torqueMsg.torque.z = torque[2]
        
        
        # msgs publications
        pubTorque.publish(torqueMsg)
         

        rate.sleep()

# -----------------------------------------------------------------------------

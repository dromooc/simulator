#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
from nav_msgs.msg import Odometry
from simulator.msg import Thrust, RPYAngles, WayPoint, Angle
import numpy as np
import quadrotorClass
from dynamic_reconfigure.server import Server
from simulator.cfg import positionCtrlCFGConfig
from visualization_msgs.msg import Marker

# node init
rospy.init_node('positionCtrl', anonymous=False)


# publishers
# -----------
# Thrust
pubThrust = rospy.Publisher('thrust', Thrust, queue_size=50)
# Attitude angles as reference for attitude control
pubRPYAnglesRef = rospy.Publisher('RPYAnglesRef', RPYAngles, queue_size=50)
# WP Marker for visu in RViz
pubWPMarker = rospy.Publisher('WPMarker', Marker,queue_size=50)

# control frequency
freq = 20.
Ts = 1./freq
rate = rospy.Rate(freq)

# quadrotor
quadrotor = quadrotorClass.Quadrotor()
# state components
pos = np.zeros((3,1))
vel = np.zeros((3,1))
# WP ref components
posRef = np.zeros((3,1))
velRef = np.zeros((3,1)) # zero for WP stabilization
# Yaw reference
yawRef = 0.


# marker object
WPMarker = Marker()
WPMarker.header.frame_id = "world"
WPMarker.header.stamp = rospy.get_rostime()
WPMarker.id = 0
WPMarker.action = Marker.ADD
WPMarker.type = Marker.SPHERE #2 # sphere
WPMarker.pose.position.x = 0.
WPMarker.pose.position.y = 0.
WPMarker.pose.position.z = 0.
WPMarker.pose.orientation.x = 0
WPMarker.pose.orientation.y = 0
WPMarker.pose.orientation.z = 0
WPMarker.pose.orientation.w = 1.0
WPMarker.scale.x = 0.1
WPMarker.scale.y = 0.1
WPMarker.scale.z = 0.1
WPMarker.color.r = 1.0
WPMarker.color.g = 1.0
WPMarker.color.b = 0.0
WPMarker.color.a = 1.0



''' dynamic parameters '''
'''
# PID gains
kp = 0.12
ki = 0.
kd = 0.48
'''

# ----------------------------------------------------------------------------
def callbackDynParam(config, level):
# -----------------------------------------------------------------------------

    global kp, ki, kd

    kp = float("""{Kp}""".format(**config))
    ki = float("""{Ki}""".format(**config))
    kd = float("""{Kd}""".format(**config))

    return config
# -----------------------------------------------------------------------------


# server for dyamic parameters
srv = Server(positionCtrlCFGConfig, callbackDynParam)

# init dynamic parameters
kp = rospy.get_param('/positionCtrl/Kp', 0.12)
ki = rospy.get_param('/positionCtrl/Ki', 0.)
kd = rospy.get_param('/positionCtrl/Kd', 0.48)




# subscribers callbacks
# ----------------------

# -----------------------------------------------------------------------------
def callBackYawRef(data):
# -----------------------------------------------------------------------------
    global yawRef
    
    #read yaw reference from message
    yawRef = data.angle
# -----------------------------------------------------------------------------        


# -----------------------------------------------------------------------------
def callBackWP(data):
# -----------------------------------------------------------------------------
    global posRef, WPMarker
    
    #read position from WP message
    posRef[0] = data.position.x
    posRef[1] = data.position.y
    posRef[2] = data.position.z
    
    WPMarker.header.stamp = rospy.Time.now()
    WPMarker.header.seq += 1
    WPMarker.pose.position.x = data.position.x
    WPMarker.pose.position.y = data.position.y
    WPMarker.pose.position.z = data.position.z
    
    pubWPMarker.publish(WPMarker)


# -----------------------------------------------------------------------------        



# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global pos, vit
    
    # read position from odom msg    
    pos[0] = data.pose.pose.position.x
    pos[1] = data.pose.pose.position.y
    pos[2] = data.pose.pose.position.z
    # read linear velocity from odom msg    
    vel[0] = data.twist.twist.linear.x
    vel[1] = data.twist.twist.linear.y
    vel[2] = data.twist.twist.linear.z
# -----------------------------------------------------------------------------        
        
   
# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("WayPoint", WayPoint, callBackWP)
rospy.Subscriber("YawRef", Angle, callBackYawRef)


# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    
    #global quadrotor

    # init messages
    thrustMsg = Thrust()
    RPYAnglesRefMsg = RPYAngles()
    
    # variables
    acc = np.zeros((3,1))
    e3 = np.array([[0.],[0.],[1.]])
    
    # main loop
    while not rospy.is_shutdown():
        

        # PID for position control
        acc = -kp*(pos - posRef) -kd*(vel - velRef)

        # Thrust vector
        Tvec = quadrotor.m * (acc + quadrotor.g0*e3)
        
        # Thrust magnitude
        T = np.linalg.norm(Tvec , 2)

        # desired angles corresponding to Thrust vector direction, with given reference yaw 
        if (T==0.):
            rollRef = 0.
            pitchRef = 0.
        
        else: # T>0

            Rd_e3 = Tvec / T
            
            #if ( np.abs(np.mod(yawRef,2.*np.pi)) == np.pi/2.):  # yaw = +/- pi/2
            
            # bring back yawRef in -pi,pi for test
            yawRefTest = np.mod(yawRef, 2.*np.pi)
            if (yawRefTest>np.pi):
                yawRefTest = yawRefTest - 2.*np.pi
            if (np.abs((yawRefTest - np.pi/2.))<0.001)or(np.abs((yawRefTest + np.pi/2.))<0.001):
            #if np.abs( np.abs(np.mod(yawRef,2.*np.pi)) -np.pi - np.pi/2.)<0.01:  # yaw = +/- pi/2               
                pitchRef = np.sign(yawRefTest) * np.arctan2(Rd_e3[1] , Rd_e3[2] )
                rollRef = np.sign(yawRefTest) * np.arctan2( np.cos(pitchRef) *  Rd_e3[0] , Rd_e3[2] )
            else:
                pitchRef = np.arctan2( np.cos(yawRef)*Rd_e3[0] + np.sin(yawRef)*Rd_e3[1], Rd_e3[2]  )            
                rollRef = np.arctan( np.sin(pitchRef)*np.tan(yawRef) - (Rd_e3[1]*np.cos(pitchRef))/(Rd_e3[2]*np.cos(yawRef)) )

    
        # msgs update        
        timeNow = rospy.Time.now()
        thrustMsg.header.seq += 1
        thrustMsg.header.stamp = timeNow     
        thrustMsg.thrust = T        
        
        RPYAnglesRefMsg.header.seq += 1
        RPYAnglesRefMsg.header.stamp = timeNow
        RPYAnglesRefMsg.roll = rollRef
        RPYAnglesRefMsg.pitch = pitchRef
        RPYAnglesRefMsg.yaw = yawRef
        
        
        # msgs publications
        pubThrust.publish(thrustMsg)
        pubRPYAnglesRef.publish(RPYAnglesRefMsg)
        
        rate.sleep()

# -----------------------------------------------------------------------------

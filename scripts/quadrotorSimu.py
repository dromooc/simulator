#!/usr/bin/env python
"""
#DroMOOC (www.onera.fr/dromooc)
#ONERA-ENSTA-CentraleSupelec-Paris Saclay
#all variables are in SI unit
"""

import rospy
import tf
from geometry_msgs.msg import Quaternion#, Wrench
from nav_msgs.msg import Odometry
from simulator.msg import Thrust, Force, Torque, MotorSpeeds, RPYAngles
import quadrotorClass



# node init
rospy.init_node('quadrotorSimu', anonymous=False)


# quadrotor object
quadrotor = quadrotorClass.Quadrotor()
T = quadrotor.g0 * quadrotor.m
Gamma1 = 0.
Gamma2 = 0.
Gamma3 = 0.
Fext_x = 0.
Fext_y = 0.
Fext_z = 0.



# publishers
# -----------
# odometry
pubOdometry = rospy.Publisher('odom', Odometry, queue_size=50)
pubRPYAngles = rospy.Publisher('RPYAngles', RPYAngles, queue_size=50)


# frequency of integration
fint = 100.
Tint = 1./fint
integrationRate = rospy.Rate(fint)


# tf broadcaster
# ---------------
odomTFBroadcaster = tf.TransformBroadcaster()


''' a passer en param statique '''
inputMode = 'ThrustAndTorques'  # 'MotorSpeeds"



# subscribers callbacks
# ----------------------


# -----------------------------------------------------------------------------
def callBackThrust(data):
# -----------------------------------------------------------------------------
    global T
    
    T = data.thrust
# -----------------------------------------------------------------------------        


# -----------------------------------------------------------------------------
def callBackTorque(data):
# -----------------------------------------------------------------------------
    global Gamma1, Gamma2, Gamma3
    
    Gamma1 = data.torque.x
    Gamma2 = data.torque.y
    Gamma3 = data.torque.z
# -----------------------------------------------------------------------------        



# -----------------------------------------------------------------------------
def callBackFext(data):
# -----------------------------------------------------------------------------
    global Fext_x, Fext_y, Fext_z
    
    Fext_x = data.force.x
    Fext_y = data.force.y
    Fext_z = data.force.z
# -----------------------------------------------------------------------------        






'''
# -----------------------------------------------------------------------------
def callBackWrench(data):
# -----------------------------------------------------------------------------
    global T, Gamma1, Gamma2, Gamma3
    
    T = data.force.z
    Gamma1 = data.torque.x
    Gamma2 = data.torque.y
    Gamma3 = data.torque.z
            
# -----------------------------------------------------------------------------        
'''        
   
# subscribers
# ------------
#rospy.Subscriber("inputWrench", Wrench, callBackWrench)
rospy.Subscriber("thrust", Thrust, callBackThrust)
rospy.Subscriber("torque", Torque, callBackTorque)
rospy.Subscriber("Fext", Force, callBackFext)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    #rospy.spin()    
    #global quadrotor

    odomMsg = Odometry()
    odomMsg.header.frame_id ='world'
    odomMsg.child_frame_id = 'quadrotor'

    RPYAnglesMsg = RPYAngles()
    
    t = rospy.get_time()
    quaternion = Quaternion()

    while not rospy.is_shutdown():
        quadrotor.stateUpdateFromInput([T, Gamma1, Gamma2, Gamma3], inputType=inputMode, FextVector=[Fext_x, Fext_y, Fext_z], Ts=Tint)
        
        quat = tf.transformations.quaternion_from_euler(quadrotor.phi, quadrotor.theta, quadrotor.psi)  # roll, pitch, yaw
        quaternion = Quaternion(*tf.transformations.quaternion_from_euler(quadrotor.phi, quadrotor.theta, quadrotor.psi))
        
        # time
        timeNow = rospy.Time.now()
        
        # broadcast TF
        odomTFBroadcaster.sendTransform( (quadrotor.x, quadrotor.y, quadrotor.z),  quat, timeNow, "quadrotor", "world")

        # odometry msgs        
        odomMsg.header.seq = odomMsg.header.seq + 1
        odomMsg.header.stamp = timeNow
         
        odomMsg.pose.pose.position.x = quadrotor.x
        odomMsg.pose.pose.position.y = quadrotor.y
        odomMsg.pose.pose.position.z = quadrotor.z
        odomMsg.pose.pose.orientation.x = quaternion.x
        odomMsg.pose.pose.orientation.y = quaternion.y
        odomMsg.pose.pose.orientation.z = quaternion.z
        odomMsg.pose.pose.orientation.w = quaternion.w
         
      
        odomMsg.twist.twist.linear.x = quadrotor.Vx
        odomMsg.twist.twist.linear.y = quadrotor.Vy
        odomMsg.twist.twist.linear.z = quadrotor.Vz
        odomMsg.twist.twist.angular.x = quadrotor.Omega_p
        odomMsg.twist.twist.angular.y = quadrotor.Omega_q       
        odomMsg.twist.twist.angular.z = quadrotor.Omega_r
        
         
        # Roll Pitch Yaw angles msg
        RPYAnglesMsg.header.seq = RPYAnglesMsg.header.seq + 1
        RPYAnglesMsg.header.stamp = timeNow

        RPYAnglesMsg.roll =  quadrotor.phi
        RPYAnglesMsg.pitch =  quadrotor.theta
        RPYAnglesMsg.yaw =  quadrotor.psi
                
         
        # msgs publications
        pubOdometry.publish(odomMsg)
        pubRPYAngles.publish(RPYAnglesMsg)


        integrationRate.sleep()

# -----------------------------------------------------------------------------

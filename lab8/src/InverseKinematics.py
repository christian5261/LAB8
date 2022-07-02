#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from mainFunctions import *

np.set_printoptions(precision=4, suppress=True)

rospy.init_node("test_ikine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker = BallMarker(color['GREEN'])
bmarker_des = BallMarker(color['RED'])

# Joint names
jnames = ['right_j0', 'right_j1', 'right_j2',
          'right_j3', 'right_j4', 'right_j5', 'right_j6']

# Desired position
xd = np.array([0.5, 0.2, 0.3])
# Initial configuration
q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Inverse kinematics
q = ikine_sawyer(xd, q0)

# Resulting position (end effector with respect to the base link)
T = fkine_sawyer(q)
print('Obtained value:')
print(T)

# Red marker shows the achieved position
bmarker.xyz(T[0:3, 3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()

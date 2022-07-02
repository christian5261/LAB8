#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from markers import *
from mainFunctions import *

np.set_printoptions(precision=4, suppress=True)

rospy.init_node("test_fkine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1)
bmarker = BallMarker(color['GREEN'])

# Joint names
jnames = ['right_j0', 'right_j1', 'right_j2', 
          'right_j3', 'right_j4', 'right_j5', 'right_j6']
# Joint Configuration
q = [0, 0, 0, 0, 0, 0, 0]

# End effector with respect to the base
T = fkine_sawyer(q)
print(T)
bmarker.position(T)

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(20)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()

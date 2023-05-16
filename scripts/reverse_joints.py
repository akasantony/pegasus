import rospy
import math
import time
import random
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import LinkStates, JointState
from gazebo_msgs.srv import GetModelState, GetLinkState
from std_srvs.srv import Empty
import tf

def reverse_joint_links(joint):
    # Get the current parent and child links
    parent_link = joint.GetParent()
    child_link = joint.GetChild()

    # Set the child link as the new parent link
    joint.SetParent(child_link)

    # Set the parent link as the new child link
    joint.SetChild(parent_link)

    # Update the joint properties
    joint.Update()

# Initialize ROS node
rospy.init_node('reverse_joint_links')

# Wait for the services to become available
rospy.wait_for_service('/gazebo/set_model_state')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/get_link_state')
rospy.wait_for_service('/gazebo/reset_simulation')

# Create the service proxies
set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

# Get the world and model objects
world = gazebo.phyics.WorldPtr()
world = gazebo.phyics.get_world('default')
model = world.get_model('my_model')

# Get the joint that you want to reverse the links for
joint = model.get_joint('my_joint')

# Reverse the parent and child links of the joint
reverse_joint_links(joint)

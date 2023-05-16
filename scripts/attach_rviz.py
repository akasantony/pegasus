import rospy
from geometry_msgs.msg import Pose,PoseStamped,Wrench
from gazebo_msgs.srv import SpawnModel,ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.msg import ModelState, ModelStates
import moveit_commander
import sys
import math

arm_right = moveit_commander.MoveGroupCommander('arm_right')
robot = moveit_commander.RobotCommander()
arm_right.set_num_planning_attempts(45)



arm_right.attach_object('object_3')
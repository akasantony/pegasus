import sys
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler
from fullbot_plugin.srv import Attach, AttachRequest, AttachResponse
from fullbot_plugin.srv import Detach, DetachRequest, DetachResponse



SCENE = moveit_commander.PlanningSceneInterface()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move')
rospy.sleep(2)

arm_right = moveit_commander.MoveGroupCommander('arm_right')
robot = moveit_commander.RobotCommander()
arm_right.set_num_planning_attempts(45)


PICK_ORIENTATION_EULER = [0,math.pi/2,0]

goal_pose = Pose()

goal_pose.position.x = 0.1
goal_pose.position.y = 0.45
goal_pose.position.z = 0.85
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.25-0.1
goal_pose.position.y = 0.45
goal_pose.position.z = 0.85
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.25
goal_pose.position.y = 0.45+0.42
goal_pose.position.z = 0.85
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

arm_right.attach_object('object_1_1')

attach_srv = rospy.ServiceProxy('/fullbot_plugin/link/attach',Attach)
attach_srv.wait_for_service()

# Link them
rospy.loginfo("Attaching robot and SRS")
req = AttachRequest()
req.parent_model = "robot"
req.parent_link = "link6_right"
req.child_model = "object_1_1"
req.child_link = "object_1_1_link"
req.joint_name = "virtual_joint"

attach_srv.call(req)

goal_pose.position.x = 0.3-0.1
goal_pose.position.y = 0.45+0.45
goal_pose.position.z = 0.8+0.45
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.3-0.1
goal_pose.position.y = 0.45+0.2
goal_pose.position.z = 0.8+0.45+0.05
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.3
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.45+0.05
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.3
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.45+0.01
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.3
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.45
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

arm_right.detach_object('object_1_1')

detach_srv = rospy.ServiceProxy('/fullbot_plugin/link/detach',Detach)
detach_srv.wait_for_service()

rospy.loginfo("Detaching robot and SRS")
req = DetachRequest()
req.detach_model = "robot"
req.detach_joint = "virtual_joint"

detach_srv.call(req)

goal_pose.position.x = 0.3 - 0.1
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.225
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

goal_pose.position.x = 0.3 - 0.1
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_right.set_pose_target(goal_pose)
arm_right.set_goal_position_tolerance(0.001)
arm_right.go(wait=True)

arm_up = moveit_commander.MoveGroupCommander('arm_up')
robot = moveit_commander.RobotCommander()
arm_up.set_num_planning_attempts(45)


PICK_ORIENTATION_EULER = [0,math.pi,0]

goal_pose.position.x = 0.5
goal_pose.position.y = 0.0
goal_pose.position.z = 0.8+0.45+0.27
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)

goal_pose.position.x = 0.5
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.45+0.3
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)

attach_srv = rospy.ServiceProxy('/fullbot_plugin/link/attach',Attach)
attach_srv.wait_for_service()

arm_up.attach_object('object_1_1')

rospy.loginfo("Attaching robot and SRS ***")
req = AttachRequest()

req.parent_model = "robot"
req.parent_link = "link6_up"
req.child_model = "object_1_1"
req.child_link = "object_1_1_link"
req.joint_name = "virtual_joint2"

attach_srv.call(req)

goal_pose.position.x = 0.45
goal_pose.position.y = 0.45
goal_pose.position.z = 0.8+0.45+0.32
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)

# goal_pose.position.x = 0.4
# goal_pose.position.y = 0.35
# goal_pose.position.z = 0.8+0.45+0.3 +0.2
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y = orientation[1]
# goal_pose.orientation.z = orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)


# goal_pose.position.x = 0.3
# goal_pose.position.y = 0.0
# goal_pose.position.z = 0.8+0.45+0.45
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y = orientation[1]
# goal_pose.orientation.z = orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)

goal_pose.position.x = 0.3 + 0.25
goal_pose.position.y = -0.45
goal_pose.position.z = 0.8+0.45+0.3
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)

goal_pose.position.x = 0.3 + 0.25
goal_pose.position.y = -0.42
goal_pose.position.z = 0.8+0.45+0.3
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)

arm_up.detach_object('object_1_1')

detach_srv = rospy.ServiceProxy('/fullbot_plugin/link/detach',Detach)
detach_srv.wait_for_service()

rospy.loginfo("Detaching robot and SRS**")
req = DetachRequest()
req.detach_model = "robot"
req.detach_joint = "virtual_joint2"

detach_srv.call(req)


attach_srv = rospy.ServiceProxy('/fullbot_plugin/link/attach',Attach)
attach_srv.wait_for_service()

rospy.loginfo("Attaching robot and SRS")
req = AttachRequest()

req.parent_model = "object_2"
req.parent_link = "srsstatic"
req.child_model = "object_1_1"
req.child_link = "object_1_1_link"
req.joint_name = "virtual_joint3"

attach_srv.call(req)

goal_pose.position.x = 0.6
goal_pose.position.y = 0.0
goal_pose.position.z = 0.8+0.45
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y = orientation[1]
goal_pose.orientation.z = orientation[2]
goal_pose.orientation.w = orientation[3]

arm_up.set_pose_target(goal_pose)
arm_up.set_goal_position_tolerance(0.001)
arm_up.go(wait=True)


arm_left = moveit_commander.MoveGroupCommander('arm_left')
arm_left.set_num_planning_attempts(45)

PICK_ORIENTATION_EULER = [0,math.pi/2,0]

goal_pose.position.x = 0.3
goal_pose.position.y = -0.45
goal_pose.position.z = 0.8+0.45
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y= orientation[1]
goal_pose.orientation.z= orientation[2]
goal_pose.orientation.w = orientation[3]

arm_left.set_pose_target(goal_pose)
arm_left.set_goal_position_tolerance(0.001)
arm_left.go(wait=True)

detach_srv = rospy.ServiceProxy('/fullbot_plugin/link/detach',Detach)
detach_srv.wait_for_service()

rospy.loginfo("Detaching robot and SRS")
req = DetachRequest()
req.detach_model = "object_2"
req.detach_joint = "virtual_joint3"

detach_srv.call(req)

arm_left.attach_object('object_1_1')

attach_srv = rospy.ServiceProxy('/fullbot_plugin/link/attach',Attach)
attach_srv.wait_for_service()

rospy.loginfo("Attaching robot and SRS")
req = AttachRequest()

req.parent_model = "robot"
req.parent_link = "link6_left"
req.child_model = "object_1_1"
req.child_link = "object_1_1_link"
req.joint_name = "virtual_joint4"

attach_srv.call(req)

goal_pose = Pose()
goal_pose = Pose()
goal_pose.position.x = 0.3
goal_pose.position.y = -0.45-0.45
goal_pose.position.z = 0.8+0.47
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y= orientation[1]
goal_pose.orientation.z= orientation[2]
goal_pose.orientation.w = orientation[3]

arm_left.set_pose_target(goal_pose)
arm_left.set_goal_position_tolerance(0.001)
arm_left.go(wait=True)

goal_pose.position.x = 0.3
goal_pose.position.y = -0.45
goal_pose.position.z = 0.8
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y= orientation[1]
goal_pose.orientation.z= orientation[2]
goal_pose.orientation.w = orientation[3]

arm_left.set_pose_target(goal_pose)
arm_left.set_goal_position_tolerance(0.001)
arm_left.go(wait=True)

arm_left.detach_object('object_1_1')

detach_srv = rospy.ServiceProxy('/fullbot_plugin/link/detach',Detach)
detach_srv.wait_for_service()

rospy.loginfo("Detaching robot and SRS")
req = DetachRequest()
req.detach_model = "robot"
req.detach_joint = "virtual_joint4"

detach_srv.call(req)


# attach_srv = rospy.ServiceProxy('/fullbot_plugin/link/attach',Attach)
# attach_srv.wait_for_service()

# # Link them
# rospy.loginfo("Attaching robot and SRS")
# req = AttachRequest()
# req.parent_model = "robot"
# req.parent_link = "link6_left"
# req.child_model = "object_1"
# req.child_link = "object_1_link"
# req.parent_child_attach_joint = "joint6"
# req.detach_model = "robot"
# req.detach_link = "link4_left"
# req.detach_joint = "left" 

# attach_srv.call(req)

# goal_pose.position.x = 0.5-0.1
# goal_pose.position.y = -0.4-0.4
# goal_pose.position.z = 0.5+0.2
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_left.set_pose_target(goal_pose)
# arm_left.set_goal_position_tolerance(0.001)
# arm_left.go(wait=True)

# goal_pose.position.x = 0.5-0.1
# goal_pose.position.y = -0.4-0.4
# goal_pose.position.z = 0.5+0.2+0.45
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_left.set_pose_target(goal_pose)
# arm_left.set_goal_position_tolerance(0.001)
# arm_left.go(wait=True)

# goal_pose = Pose()
# goal_pose = Pose()
# goal_pose.position.x = 0.5
# goal_pose.position.y = -0.4-0.4+0.4
# goal_pose.position.z = 0.5+0.2+0.45
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_left.set_pose_target(goal_pose)
# arm_left.set_goal_position_tolerance(0.001)
# arm_left.go(wait=True)

# goal_pose = Pose()
# goal_pose = Pose()
# goal_pose.position.x = 0.5
# goal_pose.position.y = -0.4-0.4+0.4
# goal_pose.position.z = 0.5+0.2+0.4
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_left.set_pose_target(goal_pose)
# arm_left.set_goal_position_tolerance(0.001)
# arm_left.go(wait=True)

# detach

# arm_up = moveit_commander.MoveGroupCommander('arm_up')
# robot = moveit_commander.RobotCommander()
# arm_up.set_num_planning_attempts(45)


# PICK_ORIENTATION_EULER = [0,math.pi,0]

# goal_pose = Pose()
# goal_pose.position.x = 0.5
# goal_pose.position.y = 0.4
# goal_pose.position.z = 0.65+0.4+0.1+0.4
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y = orientation[1]
# goal_pose.orientation.z = orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)

# arm_up.attach_object('object_12')

# PICK_ORIENTATION_EULER = [0,0,0]

# goal_pose = Pose()
# dest = [-0.5, -0.6+1*0.4 + 0.2, 1.2+0.6]
# goal_pose.position.x = dest[0]
# goal_pose.position.y = dest[1]
# goal_pose.position.z = dest[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)

# PICK_ORIENTATION_EULER = [0,0,0]

# goal_pose = Pose()
# dest = [0.5, -0.6 + 1*0.4 +0.2, 1.2+0.6]
# goal_pose.position.x = dest[0]
# goal_pose.position.y = dest[1]
# goal_pose.position.z = dest[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)

# # PICK_ORIENTATION_EULER = [math.pi,0,0]

# # goal_pose = Pose()
# # dest = [-0.5, -0.6 + 1*0.4 +0.2, 1.2+0.6]
# # goal_pose.position.x = dest[0]
# # goal_pose.position.y = dest[1]
# # goal_pose.position.z = dest[2]
# # orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# # goal_pose.orientation.x = orientation[0]
# # goal_pose.orientation.y= orientation[1]
# # goal_pose.orientation.z= orientation[2]
# # goal_pose.orientation.w = orientation[3]

# # arm_up.set_pose_target(goal_pose)
# # arm_up.set_goal_position_tolerance(0.001)
# # arm_up.go(wait=True)

# PICK_ORIENTATION_EULER = [math.pi,0,0]

# goal_pose = Pose()
# dest = [0.5, -0.6 + 0*0.4 +0.2, 1.2+0.4]
# goal_pose.position.x = dest[0]
# goal_pose.position.y = dest[1]
# goal_pose.position.z = dest[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_up.set_pose_target(goal_pose)
# arm_up.set_goal_position_tolerance(0.001)
# arm_up.go(wait=True)
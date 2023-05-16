import sys
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler

SCENE = moveit_commander.PlanningSceneInterface()
object = CollisionObject()
object.id = 'table_1'
object.header.frame_id = 'base_link'
solid = SolidPrimitive()
solid.type = solid.BOX
solid.dimensions = [0.3, 0.3, 0.2]
object.primitives = [solid]

object_pose = Pose()
pose= [0.15, -0.3, 0.4]
object_pose.position.x = pose[0]
object_pose.position.y = pose[1]
object_pose.position.z = pose[2]

object.primitive_poses = [object_pose]
object.operation = object.ADD
SCENE.add_object(object)


object_pose = PoseStamped()
pose=[0.15, -0.45, 0.6]
object_pose.header.frame_id = 'base_link'
object_pose.pose.position.x = pose[0]
object_pose.pose.position.y = pose[1]
object_pose.pose.position.z = pose[2]

# object.primitive_poses = [object_pose]
# object.operation = object.ADD
SCENE.add_mesh(name = 'object',pose = object_pose,filename = '/home/likith/ws_moveit/src/initial_config/meshes/base_link.STL')

# mesh = moveit_python.planning_scene_interface.PlanningSceneInterface()
# mesh.addMesh(name = 'object',pose = object_pose,filename = '/home/likith/ws_moveit/src/initial_config/meshes/base_link.STL',wait = True)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('gen3_lite_pick_place')
rospy.sleep(2)

arm = moveit_commander.MoveGroupCommander('kinematic')
robot = moveit_commander.RobotCommander('robot_description')
gripper = robot.get_joint('joint6')
arm.set_num_planning_attempts(45)


OBJECT_POSITIONS = [0.15, -0.15, 0.6]
PICK_ORIENTATION_EULER = [0, math.pi/2, 0*-math.pi/2]
pose = Pose()
pose.position.x = OBJECT_POSITIONS[0]
pose.position.y = OBJECT_POSITIONS[1] + 0.1
pose.position.z = OBJECT_POSITIONS[2]
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
pose.orientation.x = orientation[0]
pose.orientation.y = orientation[1]
pose.orientation.z = orientation[2]
pose.orientation.w = orientation[3]
arm.set_pose_target(pose)
arm.set_goal_position_tolerance(0.001)
arm.go(wait=True)
arm.attach_object('object')

OBJECT_POSITIONS = [0.15, -0.15, 0.6]
PICK_ORIENTATION_EULER = [0, math.pi/2, 0*-math.pi/2]
pose = Pose()
pose.position.x = OBJECT_POSITIONS[0]
pose.position.y = OBJECT_POSITIONS[1] + 0.05
pose.position.z = OBJECT_POSITIONS[2]
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
pose.orientation.x = orientation[0]
pose.orientation.y = orientation[1]
pose.orientation.z = orientation[2]
pose.orientation.w = orientation[3]
arm.set_pose_target(pose)
arm.set_goal_position_tolerance(0.001)
arm.go(wait=True)
arm.detach_object('object')

import sys
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler

def set_objects(SCENE,rospy):
    for i in range(20):
        object_pose = PoseStamped()
        pose=[-0.75+i*0.5, -0.6, 0.9]
        object_pose.header.frame_id = 'world'
        object_pose.pose.position.x = pose[0]
        object_pose.pose.position.y = pose[1]
        object_pose.pose.position.z = pose[2]

        SCENE.add_mesh(name = 'object_'+str(i+1),pose = object_pose,filename = '/home/likith/ws_moveit/src/mario_mov/meshes/base_link.STL')

SCENE = moveit_commander.PlanningSceneInterface()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move')
rospy.sleep(2)

arm_left = moveit_commander.MoveGroupCommander('arm_left')
robot = moveit_commander.RobotCommander()
gripper = robot.get_joint('joint6_left')
arm_left.set_num_planning_attempts(45)

set_objects(SCENE,rospy)

current_pose = arm_left.get_current_pose('link6_up_eft').pose.position
current_orientation = arm_left.get_current_pose('link6_up_eft').pose.orientation
print(current_pose)
print(current_orientation)

PICK_ORIENTATION_EULER = [-math.pi/2,0,0]

goal_pose = Pose()
dest = [-0.75+3*0.5, -0.6+0.25, 0.9+0.25]
goal_pose.position.x = dest[0]
goal_pose.position.y = dest[1]
goal_pose.position.z = dest[2]
orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
goal_pose.orientation.x = orientation[0]
goal_pose.orientation.y= orientation[1]
goal_pose.orientation.z= orientation[2]
goal_pose.orientation.w = orientation[3]

arm_left.set_pose_target(goal_pose)
arm_left.set_goal_position_tolerance(0.001)
arm_left.go(wait=True)
# arm_left.attach_object('object_1')


# arm_left = moveit_commander.MoveGroupCommander('arm_left')
# PICK_ORIENTATION_EULER = [-math.pi/2,0,0]

# goal_pose = Pose()
# dest = [-0.75+0.5*3,-0.65,0.4+0.25]
# goal_pose.position.x = dest[0]
# goal_pose.position.y = dest[1]
# goal_pose.position.z = dest[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# goal_pose.orientation.x = orientation[0]
# goal_pose.orientation.y= orientation[1]
# goal_pose.orientation.z= orientation[2]
# goal_pose.orientation.w = orientation[3]

# arm_left.set_pose_target(goal_pose)
# arm_left.set_goal_position_tolerance(0.001)
# arm_left.go(wait=True)
# arm_left.attach_object('object_4')

# object = CollisionObject()
# object.id = 'table_1'
# object.header.frame_id = ''
# solid = SolidPrimitive()
# solid.type = solid.BOX
# solid.dimensions = [0.3, 0.3, 0.2]
# object.primitives = [solid]

# object_pose = Pose()
# pose= [0.15, -0.3, 0.4]
# object_pose.position.x = pose[0]
# object_pose.position.y = pose[1]
# object_pose.position.z = pose[2]

# object.primitive_poses = [object_pose]
# object.operation = object.ADD
# SCENE.add_object(object)

# arm = moveit_commander.MoveGroupCommander('kinematic')
# robot = moveit_commander.RobotCommander('robot_description')
# gripper = robot.get_joint('joint6')
# arm.set_num_planning_attempts(45)


# OBJECT_POSITIONS = [0.15, -0.15, 0.6]
# PICK_ORIENTATION_EULER = [0, math.pi/2, 0*-math.pi/2]
# pose = Pose()
# pose.position.x = OBJECT_POSITIONS[0]
# pose.position.y = OBJECT_POSITIONS[1] + 0.1
# pose.position.z = OBJECT_POSITIONS[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# pose.orientation.x = orientation[0]
# pose.orientation.y = orientation[1]
# pose.orientation.z = orientation[2]
# pose.orientation.w = orientation[3]
# arm.set_pose_target(pose)
# arm.set_goal_position_tolerance(0.001)
# arm.go(wait=True)
# arm.attach_object('object')

# OBJECT_POSITIONS = [0.15, -0.15, 0.6]
# PICK_ORIENTATION_EULER = [0, math.pi/2, 0*-math.pi/2]
# pose = Pose()
# pose.position.x = OBJECT_POSITIONS[0]
# pose.position.y = OBJECT_POSITIONS[1] + 0.05
# pose.position.z = OBJECT_POSITIONS[2]
# orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
# pose.orientation.x = orientation[0]
# pose.orientation.y = orientation[1]
# pose.orientation.z = orientation[2]
# pose.orientation.w = orientation[3]
# arm.set_pose_target(pose)
# arm.set_goal_position_tolerance(0.001)
# arm.go(wait=True)
# arm.detach_object('object')

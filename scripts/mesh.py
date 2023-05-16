
import sys
import math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler

def set_objects(SCENE,rospy):
    for i in range(10):
        object_pose = PoseStamped()
        pose=[0.5, -0.6+i*0.4, 0.8]
        object_pose.header.frame_id = 'world'
        object_pose.pose.position.x = pose[0]
        object_pose.pose.position.y = pose[1]
        object_pose.pose.position.z = pose[2]

        SCENE.add_mesh(name = 'object_'+str(i+1),pose = object_pose,filename = '/home/vaspan/Downloads/base_link.STL')

SCENE = moveit_commander.PlanningSceneInterface()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move')
rospy.sleep(2)

robot = moveit_commander.RobotCommander()


set_objects(SCENE,rospy)



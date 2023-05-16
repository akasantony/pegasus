import rospy
from gazebo_msgs.msg import ModelStates,LinkStates,LinkState
from geometry_msgs.msg import Pose,Wrench, Vector3
import numpy as np

from gazebo_msgs.srv import ApplyJointEffort, ApplyJointEffortRequest


obj_position = None
link_position = None

def model_state_callback(data):
    # Get the index of the first link of the robot
    # link_index = data.name.index('fullbot')
    # print(len(data.pose))

    # # Get the current state of the first link
    # link_state = data.pose[link_index]
    # print(link_state)

    # Create a wrench_1 to apply a force to the link
    wrench_1 = ApplyBodyWrenchRequest()
    wrench_1.body_name = 'object_1::object_1_link'

    wrench_1.reference_frame = 'world'
    wrench_1.wrench.force.x = 100000  # N
    wrench_1.wrench.force.y = 0
    wrench_1.wrench.force.z = 1

    wrench_2 = ApplyBodyWrenchRequest()
    wrench_2.body_name = 'object_2::object_1_link'

    wrench_2.reference_frame = 'world'
    wrench_2.wrench.force.x = 100000  # N
    wrench_2.wrench.force.y = 0
    wrench_2.wrench.force.z = 1

    # Apply the wrench_1 to the link
    apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    apply_wrench(wrench_1)
    apply_wrench(wrench_2)

rospy.init_node('apply_force_to_robot_link')
rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
rospy.spin()



def distance_check():
    global obj_position, link_position
    if obj_position is not None and link_position is not None:
        point1 = np.array((obj_position.position.x,obj_position.position.y,obj_position.position.z))
        point2 = np.array((link_position.position.x,link_position.position.y,link_position.position.z))
        dist = np.linalg.norm(point1 - point2)
        print(dist)
        if dist < 1:
            apply_force(dist)

rospy.init_node('apply_force')

rospy.Subscriber('/gazebo/model_states',ModelStates,model_pose)
rospy.Subscriber('/gazebo/link_states',LinkStates,link_pose)
distance_check()
link_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)

while not rospy.is_shutdown():
    distance_check()
    rospy.sleep(0.1)
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest


def model_state_callback(data):
    # Get the index of the first link of the robot
    # link_index = data.name.index('fullbot')
    # print(len(data.pose))

    # # Get the current state of the first link
    # link_state = data.pose[link_index]
    # print(link_state)

    # Create a wrench_1 to apply a force to the link
    global apply_wrench
    wrench_1 = ApplyBodyWrenchRequest()
    wrench_1.body_name = 'object_1::object_1_link'

    wrench_1.reference_frame = 'world'
    wrench_1.wrench.force.x = -1000  # N
    wrench_1.wrench.force.y = 0
    wrench_1.wrench.force.z = 0

    wrench_2 = ApplyBodyWrenchRequest()
    wrench_2.body_name = 'object_2::object_2_link'

    wrench_2.reference_frame = 'world'
    wrench_2.wrench.force.x = 10  # N
    wrench_2.wrench.force.y = 0
    wrench_2.wrench.force.z = 0

    # Apply the wrench_1 to the link
    apply_wrench(wrench_1)
    # apply_wrench(wrench_2)

apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

rospy.init_node('apply_force_to_robot_link')
rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
# rospy.spin()

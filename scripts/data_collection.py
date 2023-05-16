import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
import pandas as pd
import os.path as path


def model_states_callback(data):
    global data_collection_pos
    global data_collection_vel
    index_ = data.name.index("object_1_1")
    position_ = data.pose[index_].position
    linear_vel_ = data.twist[index_].linear
    file_name_pos = "pos_mass_2.csv"
    file_name_vel = "vel_mass_2.csv"

    if path.exists(file_name_pos):
        data_collection_pos = pd.read_csv(file_name_pos,index_col=0)
        data_collection_vel = pd.read_csv(file_name_vel,index_col=0)
    else:
        columns = ['x','y','z']
        data_collection_pos = pd.DataFrame(columns=columns)
        data_collection_vel = pd.DataFrame(columns=columns)

    df_extended_pos = pd.DataFrame([[position_.x,position_.y,position_.z]], columns=['x', 'y', 'z'],)
    df_extended_vel = pd.DataFrame([[linear_vel_.x,linear_vel_.y,linear_vel_.z]], columns=['x', 'y', 'z'],)

    data_collection_pos = pd.concat([data_collection_pos, df_extended_pos])
    data_collection_pos.to_csv(file_name_pos)

    data_collection_vel = pd.concat([data_collection_vel, df_extended_vel])
    data_collection_vel.to_csv(file_name_vel)

    

# def joint_state_callback(data):
#     print(data)

rospy.init_node('joint_state_subscriber')
# rospy.Subscriber('/joint_states', JointState, joint_state_callback)
rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

while not rospy.is_shutdown():
    rospy.spin()
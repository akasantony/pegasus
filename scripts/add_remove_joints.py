import rospy
from geometry_msgs.msg import Pose,PoseStamped,Wrench
from gazebo_msgs.srv import SpawnModel,ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.msg import ModelState, ModelStates
import moveit_commander
import sys

rospy.init_node('ApplyBodyWrench')
apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

wrench_msg = Wrench()
wrench_msg.force.x = 0  # replace with your desired force in x direction
wrench_msg.force.y =-10 # replace with your desired force in y direction
wrench_msg.force.z = 0  # replace with your desired force in z direction
wrench_msg.torque.x = 0  # replace with your desired torque in x direction
wrench_msg.torque.y = 0  # replace with your desired torque in y direction
wrench_msg.torque.z = 0  # replace with your desired torque in z direction
apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

msg1 = ApplyBodyWrenchRequest()
msg1.body_name = 'object_'+str(3+1)+'::'+'object_'+str(3+1)+'_link' 
msg1.reference_frame = 'object_'+str(2+1)+'::'+'object_'+str(2+1)+'_link' 
msg1.wrench = wrench_msg
msg1.start_time = rospy.Time.now()
msg1.duration = rospy.Duration.from_sec(3.0) 
response = apply_wrench(msg1)






import rospy
from geometry_msgs.msg import Pose,PoseStamped,Wrench
from gazebo_msgs.srv import SpawnModel,ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.msg import ModelState, ModelStates
import moveit_commander
import sys

rospy.init_node('spawn_model')
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
SCENE = moveit_commander.PlanningSceneInterface()
moveit_commander.roscpp_initialize(sys.argv)

# 1300/800

def set_objects(name,model_pose,file_name):
    object_pose = PoseStamped()
    object_pose.header.frame_id = 'world'
    object_pose.pose = model_pose

    SCENE.add_mesh(name = name,pose = object_pose,filename = file_name)

def spawn_srs_module(name,model_pose):
    with open('/home/likith/ws_moveit/src/fullbot_moveit/urdf/srsstatic.urdf', 'r') as f:
        model_xml = f.read()
    
    resp = spawn_model(name,model_xml,'robot',model_pose,'world')
    print(resp)

def spawn_srs_table(name,model_pose):
    with open('/home/likith/ws_moveit/src/fullbot_moveit/urdf/srsmoduleholdingstructure.urdf', 'r') as f:
        model_xml = f.read()
    
    resp = spawn_model(name,model_xml,'robot',model_pose,'world')
    print(resp)

def plane_table(name,model_pose):
    with open('/home/likith/ws_moveit/src/fullbot_moveit/urdf/table.urdf', 'r') as f:
        model_xml = f.read()
    
    resp = spawn_model(name,model_xml,'robot',model_pose,'world')
    print(resp)
# def add_model(name,model_pose):
#     mesh_data = ''

#     sdf_template = """
#     <sdf version="1.6">
#     <model name="{name}">
#         <link name="{name}_link">
#         <collision name="{name}_collision">
#             <geometry>
#             <mesh><uri>{mesh_uri}</uri></mesh>
#             </geometry>
#         </collision>
#         <visual name="{name}_visual">
#             <geometry>
#             <mesh><uri>{mesh_uri}</uri></mesh>
#             </geometry>
#         </visual>
#         </link>
#     </model>
#     </sdf>
#     """

#     # define the variable values
#     mesh_uri = "file:///home/likith/ws_moveit/src/fullbot_moveit/meshes/baselink.STL"

#     # format the SDF template with the variable values
#     sdf_string = sdf_template.format(name=name, mesh_uri=mesh_uri)
#     # model_state = ModelState()
#     # model_state.model_name = name
#     # model_state.model_xml = sdf_string


#     # spawn the model
#     resp = spawn_model(name,sdf_string,'robot',model_pose,'world')
#     print(resp)

# def model_state_callback(name):
#     # Create a wrench_1 to apply a force to the link
#     global apply_wrench
#     wrench_1 = ApplyBodyWrenchRequest()
#     wrench_1.body_name = name

#     wrench_1.reference_frame = 'world'
#     wrench_1.wrench.force.x = -1000  # N
#     wrench_1.wrench.force.y = 0
#     wrench_1.wrench.force.z = 0

#     # Apply the wrench_1 to the link
#     apply_wrench(wrench_1)

def add_wall(name,model_pose):
    mesh_data = ''

    sdf_template = """
    <sdf version="1.6">
    <model name="{name}">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>1.8e-7</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.8e-7</iyy>
            <iyz>0</iyz>
            <izz>1.8e-7</izz>
          </inertia>
        </inertial>
        <link name="{name}_link">
        <pose>0 0 0.25 0 0 0</pose>
        <collision name="{name}_collision">
            <geometry>
                <box>
                    <size>0.5 2.5 0.5</size>
                </box>
            </geometry>
        </collision>
        <visual name="{name}_visual">
            <geometry>
                <box>
                    <size>0.5 2.5 0.5</size>
                </box>
            </geometry>
        </visual>
        </link>
        <joint name="box_world_joint" type="fixed">
            <parent>world</parent>
            <child>{name}_link</child>
            <pose>0 0 0 0 0 0</pose>
        </joint>
    </model>
    </sdf>
    """

    # define the variable values
    mesh_uri = "file:///home/likith/ws_moveit/src/fullbot_moveit/meshes/baselink.STL"

    # format the SDF template with the variable values
    sdf_string = sdf_template.format(name=name, mesh_uri=mesh_uri)
    resp = spawn_model(name,sdf_string,'robot',model_pose,'world')
    print(resp)
   


def add_model_box(name,model_pose):
    mesh_data = ''
# <static>true</static>
    sdf_template = """
    <sdf version="1.6">
    <model name="{name}">
        
        <link name="{name}_link">
         <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.0001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0001</iyy>
            <iyz>0</iyz>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
        <pose>0 0 0.2 0 0 0</pose>
        <collision name="{name}_collision">
            <pose>0 0 0 0 0 0</pose>
            <geometry>
                <box>
                    <size>0.4 0.4 0.4</size>
                </box>
            </geometry>
        </collision>
        <visual name="{name}_visual">
        <pose>0 0 0 0 0 0</pose>
            <geometry>
                <box>
                    <size>0.4 0.4 0.4</size>
                </box>
            </geometry>
        </visual>
        </link>
    </model>

    </sdf>
    """

        #     <link name="small_box">
        #     <pose>-0.275 0 0.25 0 0 0</pose>
        #     <inertial>
        #         <mass>2.5</mass>
        #         <inertia>
        #         <ixx>0.0000067</ixx>
        #         <ixy>0.0</ixy>
        #         <ixz>0.0</ixz>
        #         <iyy>0.0000067</iyy>
        #         <iyz>0.0</iyz>
        #         <izz>0.0000067</izz>
        #         </inertia>
        #     </inertial>
        #     <collision name="small_box_collision">
        #         <geometry>
        #         <box>
        #             <size>0.05 0.05 0.05</size>
        #         </box>
        #         </geometry>
        #     </collision>
        #     <visual name="small_box_visual">
        #         <geometry>
        #         <box>
        #             <size>0.05 0.05 0.05</size>
        #         </box>
        #         </geometry>
        #     </visual>
        # </link>

        # <joint name="fixed_joint" type="fixed">
        #     <parent link="{name}_link"/>
        #     <child link="small_box"/>
        #     <pose>1.0 0 0 0 0 0</pose>
        # </joint>
        # <plugin name="dipole_magnet" filename="libstorm_gazebo_ros_magnet.so">
        #     <bodyName>small_box</bodyName>
        #     <dipole_moment>2200 0 0</dipole_moment>
        #     <xyzOffset>-0.1 0 0</xyzOffset>
        #     <rpyOffset>0 0 0 </rpyOffset>
        # </plugin>

        # <link name = "world" />
        # <joint name="fixed_joint" type="fixed">
        #     <parent link="world"/>
        #     <child link="{name}_link"/>
        #     <pose>1.0 0 0 0 0 0</pose>
        # </joint>

    # define the variable values
    mesh_uri = "file:///home/likith/ws_moveit/src/fullbot_moveit/meshes/baselink.STL"

    # format the SDF template with the variable values
    sdf_string = sdf_template.format(name=name, mesh_uri=mesh_uri)
    # model_state = ModelState()
    # model_state.model_name = name
    # model_state.model_xml = sdf_string


    # spawn the model
    resp = spawn_model(name,sdf_string,'robot',model_pose,'world')

    print(resp)





# add_wall('wall',model_pose)
# set_objects('wall',model_pose)

model_pose = Pose()
model_pose.position.x = 0.6
model_pose.position.y =0.45+0.45
model_pose.position.z = 0.65
model_pose.orientation.x = 0
model_pose.orientation.y = 0
model_pose.orientation.z = 0
model_pose.orientation.w = 1

spawn_srs_table('table',model_pose)
set_objects('table',model_pose,'/home/likith/ws_moveit/src/fullbot_moveit/meshes/srsholdingstructure.STL')
model_pose.position.y = -1.15
model_pose.position.z = 0.0
plane_table('table_1',model_pose)
set_objects('table_1',model_pose,'/home/likith/ws_moveit/src/fullbot_moveit/meshes/table.STL')
for i in range(0,2):
    model_pose = Pose()
    model_pose.position.x = 0.6
    model_pose.position.y = (-i)*0.45
    model_pose.position.z = 0.65
    model_pose.orientation.x = 0
    model_pose.orientation.y = 0
    model_pose.orientation.z = 0
    model_pose.orientation.w = 1

    name = 'object_'+str(i+1)
    # spawn_srs_module(name,model_pose)
    add_model_box(name,model_pose)
    set_objects(name,model_pose,'/home/likith/ws_moveit/src/fullbot_moveit/meshes/srsstatic.STL')

model_pose = Pose()
model_pose.position.x = 0.6
model_pose.position.y = 0.45+0.45
model_pose.position.z = 0.65
model_pose.orientation.x = 0
model_pose.orientation.y = 0
model_pose.orientation.z = 0
model_pose.orientation.w = 1

name = 'object_1_'+str(1)
# spawn_srs_module(name,model_pose)
add_model_box(name,model_pose)
set_objects(name,model_pose,'/home/likith/ws_moveit/src/fullbot_moveit/meshes/srsstatic.STL')

# wrench_msg = Wrench()
# wrench_msg.force.x = 0  # replace with your desired force in x direction
# wrench_msg.force.y = -0.05 # replace with your desired force in y direction
# wrench_msg.force.z = 0  # replace with your desired force in z direction
# wrench_msg.torque.x = 0  # replace with your desired torque in x direction
# wrench_msg.torque.y = 0  # replace with your desired torque in y direction
# wrench_msg.torque.z = 0  # replace with your desired torque in z direction
# apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

# msg1 = ApplyBodyWrenchRequest()
# msg1.body_name = 'object_'+str(4+1)+'::'+'object_'+str(4+1)+'_link' 
# msg1.reference_frame = "world"
# msg1.wrench = wrench_msg
# msg1.start_time = rospy.Time.now()
# msg1.duration = rospy.Duration.from_sec(10000.0) 
# response = apply_wrench(msg1)
# rospy.spin()






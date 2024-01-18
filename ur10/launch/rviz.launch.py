from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	
	robot_model = 'ur10'

	xacro_file = get_package_share_directory('ur10') + '/urdf/'+ robot_model +'.urdf.xacro'

	# Launch RViz
	rviz_config_file = get_package_share_directory('ur10') + "/rviz/rviz2.rviz"
	rviz_node = Node(package    ='rviz2',
					 executable ='rviz2',
					 name       ='rviz2',
					 output     ='log',
					 arguments  =['-d', rviz_config_file])

	# Robot State Publisher sim time
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	# Start Robot State Publisher 
	robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								#  name       ='robot_state_publisher',
								#  output     ='both',
                                 parameters=[{
									'use_sim_time': use_sim_time,
                                    'robot_description': ParameterValue(
                                        Command(['xacro ', str(xacro_file)]), value_type=str)}])

	# Joint State Publisher gui
	joint_state_publisher_gui = Node(package  ='joint_state_publisher_gui',
									executable='joint_state_publisher_gui',
									output    ='screen',
									name      ='joint_state_publisher_gui')
	# RVIZ Configuration
	rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("ur10"), "rviz", "rviz2.rviz"]
    )
	
	rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
	
	# # Static TF Transform
	# tf=Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/world'  ],
    # )

	return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz_node])
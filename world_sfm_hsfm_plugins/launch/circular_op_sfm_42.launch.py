import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    world_file_name = 'circular_op_sfm_42.world'
    pkg_dir = get_package_share_directory('world_sfm_hsfm_plugins')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    config = os.path.join(pkg_dir,'config','circular_op_sfm_42.yaml')
    agents_loader=Node(
        package = 'world_sfm_hsfm_plugins',
        name = 'agent_params_loader',
        executable = 'load_agent_params',
        parameters = [config])
    spawn_entity = Node(package='world_sfm_hsfm_plugins', executable='spawn_robot.py',
                        arguments=['pioneer3at', 'demo', '0.0', '6.0', '0.0'],
                        output='screen')
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    return LaunchDescription([
        spawn_entity,
        agents_loader,
        gazebo])
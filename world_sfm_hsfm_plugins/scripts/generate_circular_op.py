#!/usr/bin/env python3

"""
Generates parameters and world file for a circular open problem with the specified:
- Number of actors
- SFM or HSFM
- Radius of the circle (between 3, 5, 6 or 7 meters)

Author:
  - Tommaso Van Der Meer
  - tommaso.vandermeer@student.unisi.it
"""
import math
import os 
import sys

pkg_dir = os.path.join(os.path.expanduser('~'),'ros2_ws/src/World-SFM-HSFM-Plugins/world_sfm_hsfm_plugins/')
config_dir = os.path.join(pkg_dir,'config/')
world_dir = os.path.join(pkg_dir,'worlds/')
launch_dir = os.path.join(pkg_dir,'launch/')

def to_degree(angle):
    angle = angle * 180 / math.pi
    return angle

def bound_angle(angle):
    if (angle > math.pi): angle -= 2 * math.pi
    if (angle < -math.pi): angle += 2 * math.pi
    return angle

def write_yaml(radius, model, pos, yaw, goal, title):
    ## DATA
    n = len(yaw) - 1
    ## SFM
    if (not model):
        data = ("agent_params_loader:\n" + 
                "   ros__parameters:\n" + 
                "       sampling_time: 0.033\n" + 
                "       runge_kutta_45: True\n" +
                "       ground_height: -0.1\n" +
                "       robot_control: True\n" +
                "       robot_name: \"pioneer3at\"\n" +
                "       laser_sensor_name: \"laser\"\n" +
                "       robot_radius: 0.3\n" +
                "       robot_mass: 35\n" +
                "       robot_goal_weight: 2.0\n" +
                "       robot_obstacle_weight: 10.0\n" +
                "       robot_social_weight: 15.0\n" +
                "       robot_velocity: 0.9\n" +
                f"       robot_initial_orientation: {yaw[0]}\n" +
                f"       robot_waypoints: [{goal[0][0]},{goal[0][1]},{pos[0][0]},{pos[0][1]}]\n" +
                f"       robot_initial_position: [{pos[0][0]},{pos[0][1]}]\n" +
                "       robot_ignore_obstacles: [\"circular_workspace\"]\n" +
                "       attach_cylinder_to_humans: True\n" +
                "       consider_robot: True\n" +
                "       agents: [")

        for i in range(n):
            if (i < n-1): data += f"\"agent{i+1}\","
            else: data += f"\"agent{i+1}\"]\n"
        
        for i in range(n):
            data += (f"       agent{i+1}:\n" +
                    f"           waypoints: [{-pos[i+1][0]},{-pos[i+1][1]},{pos[i+1][0]},{pos[i+1][1]}]\n" +
                    f"           initial_orientation: {yaw[i+1]}\n" +
                    f"           initial_position: [{pos[i+1][0]},{pos[i+1][1]}]\n" +
                    "           radius: 0.3\n" +
                    "           mass: 75\n" +
                    "           goal_weight: 2.0\n" +
                    "           obstacle_weight: 10.0\n" +
                    "           social_weight: 15.0\n" +
                    "           group_gaze_weight: 0.0\n" +
                    "           group_coh_weight: 0.0\n" +
                    "           group_rep_weight: 0.0\n" +
                    "           velocity: 0.9\n" +
                    "           animation_factor: 5.1\n" +
                    "           animation_name: \"walking\"\n" +
                    "           people_distance: 6.0\n" +
                    "           ignore_obstacles: [\"circular_workspace\"]\n" +
                    "           publish_forces: False\n")

        with open(f'{config_dir}{title}.yaml', 'w') as file:
            file.write(data)

    ## HSFM
    else:
        data = ("agent_params_loader:\n" + 
                "   ros__parameters:\n" + 
                "       sampling_time: 0.033\n" + 
                "       runge_kutta_45: True\n" +
                "       ground_height: -0.1\n" +
                "       robot_control: True\n" +
                "       robot_name: \"pioneer3at\"\n" +
                "       laser_sensor_name: \"laser\"\n" +
                "       robot_radius: 0.3\n" +
                "       robot_mass: 35\n" +
                "       robot_velocity: 0.9\n" +
                "       robot_relaxation_time: 0.5\n" +
                "       robot_k_orthogonal: 1.0\n" +
                "       robot_k_damping: 500.0\n" +
                "       robot_k_lambda: 0.3\n" +
                "       robot_alpha: 3.0\n" +
                "       robot_Ai: 2000.0\n" +
                "       robot_Aw: 2000.0\n" +
                "       robot_Bi: 0.08\n" +
                "       robot_Bw: 0.08\n" +
                "       robot_k1: 120000.0\n" +
                "       robot_k2: 240000.0\n" +
                f"       robot_initial_orientation: {yaw[0]}\n" +
                f"       robot_waypoints: [{goal[0][0]},{goal[0][1]},{pos[0][0]},{pos[0][1]}]\n" +
                f"       robot_initial_position: [{pos[0][0]},{pos[0][1]}]\n" +
                "       robot_ignore_obstacles: [\"circular_workspace\"]\n" +
                "       attach_cylinder_to_humans: True\n" +
                "       consider_robot: True\n" +
                "       agents: [")
        
        for i in range(n):
            if (i < n-1): data += f"\"agent{i+1}\","
            else: data += f"\"agent{i+1}\"]\n"

        for i in range(n):
            data += (f"       agent{i+1}:\n" +
                    f"           waypoints: [{-pos[i+1][0]},{-pos[i+1][1]},{pos[i+1][0]},{pos[i+1][1]}]\n" +
                    f"           initial_orientation: {yaw[i+1]}\n" +
                    f"           initial_position: [{pos[i+1][0]},{pos[i+1][1]}]\n" +
                    "           radius: 0.3\n" +
                    "           mass: 75\n" +
                    "           relaxation_time: 0.5\n" +
                    "           k_orthogonal: 1.0\n" +
                    "           k_damping: 500.0\n" +
                    "           k_lambda: 0.3\n" +
                    "           alpha: 3.0\n" +
                    "           group_distance_forward: 2.0\n" +
                    "           group_distance_orthogonal: 1.0\n" +
                    "           k1g: 200.0\n" +
                    "           k2g: 200.0\n" +
                    "           Ai: 2000.0\n" +
                    "           Aw: 2000.0\n" +
                    "           Bi: 0.08\n" +
                    "           Bw: 0.08\n" +
                    "           k1: 120000.0\n" +
                    "           k2: 240000.0\n" +
                    "           velocity: 0.9\n" +
                    "           animation_factor: 5.1\n" +
                    "           animation_name: \"walking\"\n" +
                    "           people_distance: 6.0\n" +
                    "           ignore_obstacles: [\"circular_workspace\"]\n" +
                    "           publish_forces: False\n")

        with open(f'{config_dir}{title}.yaml', 'w') as file:
            file.write(data)

def write_world(radius, model, n, title):
    ## SFM
    if (not model):
        data = ("<?xml version='1.0' encoding='utf-8'?>\n" +
                "<sdf version=\"1.6\">\n" +
                "   <world name=\"world\">\n" +
                "       <plugin name=\"world_sfm\" filename=\"libWorldSFMPlugin.so\">\n" +
                "       </plugin>\n" +
                "       <gui>\n" +
                "           <camera name='gzclient_camera'>\n" +
                "               <pose>-4.70385 10.895 16.2659 -0 0.921795 -1.12701</pose>\n" +
                "           </camera>\n" +
                "       </gui>\n" +
                "       <gravity>0 0 -9.8</gravity>\n" +
                "       <physics default=\"0\" name=\"default_physics\" type=\"ode\">\n" +
                "           <max_step_size>0.001</max_step_size>\n" +
                "           <real_time_factor>1</real_time_factor>\n" +
                "           <real_time_update_rate>1000</real_time_update_rate>\n" +
                "       </physics>\n" +
                "       <scene>\n" +
                "           <ambient>0.6 0.6 0.6 1</ambient>\n"
                "           <background>0.4 0.4 0.4 1</background>\n"
                "           <shadows>false</shadows>\n"
                "       </scene>\n" +
                "       <include>\n" +
                "           <uri>model://sun</uri>\n" +
                "       </include>\n" +
                "       <include>\n" +
                "           <uri>model://ground_plane</uri>\n" +
                "           <pose>0 0 -0.1 0 -0 0</pose>\n" +
                "       </include>\n" +
                "       <model name=\"circular_workspace\">\n" +
                "           <allow_auto_disable>false</allow_auto_disable>\n" +
                "           <static>true</static>\n" +
                "           <pose>0 0 -0.1 0 0 0</pose>\n" +
                "           <static>true</static>\n" +
                "           <link name=\"CW_link\">\n"
                "               <collision name=\"CW_collision\">\n" +
                "                   <pose>0 0 0 0 0 0</pose>\n" +
                "                   <geometry>\n" +
                "                       <mesh>\n" +
                f"                       <uri>model://circular_workspace_{radius}m.dae</uri>\n" +
                "                       <scale>1 1 1</scale>\n" +
                "                       </mesh>\n" +
                "                   </geometry>\n" +
                "               </collision>\n" +
                "               <visual name=\"CW_visual\">\n" +
                "                   <pose>0 0 0 0 0 0</pose>\n" +
                "                   <geometry>\n" +
                "                       <mesh>\n" +
                f"                           <uri>model://circular_workspace_{radius}m.dae</uri>\n" +
                "                           <scale>1 1 1</scale>\n" +
                "                       </mesh>\n" +
                "                   </geometry>\n" +
                "               </visual>\n" +
                "           </link>\n" +
                "       </model>\n")
        
        for i in range(n):
            data += (f"       <actor name=\"agent{i+1}\">\n" +
                     f"          <pose>0 {i} 1.01 0 0 0</pose>\n" +
                     "          <skin>\n" +
                     "              <filename>stand.dae</filename>\n" +
                     "              <scale>1.0</scale>\n" +
                     "          </skin>\n" +
                     "          <animation name=\"walking\">\n" +
                     "              <filename>walk.dae</filename>\n" +
                     "              <scale>1.000000</scale>\n" +
                     "              <interpolate_x>true</interpolate_x>\n" +
                     "          </animation>\n" +
                     "       </actor>\n")
            
        data += ("   </world>\n" + "</sdf>")

        with open(f'{world_dir}{title}.world', 'w') as file:
            file.write(data)

    ## HSFM
    else:
        data = ("<?xml version='1.0' encoding='utf-8'?>\n" +
                "<sdf version=\"1.6\">\n" +
                "   <world name=\"world\">\n" +
                "       <plugin name=\"world_sfm\" filename=\"libWorldHSFMPlugin.so\">\n" +
                "       </plugin>\n" +
                "       <gui>\n" +
                "           <camera name='gzclient_camera'>\n" +
                "               <pose>-4.70385 10.895 16.2659 -0 0.921795 -1.12701</pose>\n" +
                "           </camera>\n" +
                "       </gui>\n" +
                "       <gravity>0 0 -9.8</gravity>\n" +
                "       <physics default=\"0\" name=\"default_physics\" type=\"ode\">\n" +
                "           <max_step_size>0.001</max_step_size>\n" +
                "           <real_time_factor>1</real_time_factor>\n" +
                "           <real_time_update_rate>1000</real_time_update_rate>\n" +
                "       </physics>\n" +
                "       <scene>\n" +
                "           <ambient>0.6 0.6 0.6 1</ambient>\n"
                "           <background>0.4 0.4 0.4 1</background>\n"
                "           <shadows>false</shadows>\n"
                "       </scene>\n" +
                "       <include>\n" +
                "           <uri>model://sun</uri>\n" +
                "       </include>\n" +
                "       <include>\n" +
                "           <uri>model://ground_plane</uri>\n" +
                "           <pose>0 0 -0.1 0 -0 0</pose>\n" +
                "       </include>\n" +
                "       <model name=\"circular_workspace\">\n" +
                "           <allow_auto_disable>false</allow_auto_disable>\n" +
                "           <static>true</static>\n" +
                "           <pose>0 0 -0.1 0 0 0</pose>\n" +
                "           <static>true</static>\n" +
                "           <link name=\"CW_link\">\n"
                "               <collision name=\"CW_collision\">\n" +
                "                   <pose>0 0 0 0 0 0</pose>\n" +
                "                   <geometry>\n" +
                "                       <mesh>\n" +
                f"                       <uri>model://circular_workspace_{radius}m.dae</uri>\n" +
                "                       <scale>1 1 1</scale>\n" +
                "                       </mesh>\n" +
                "                   </geometry>\n" +
                "               </collision>\n" +
                "               <visual name=\"CW_visual\">\n" +
                "                   <pose>0 0 0 0 0 0</pose>\n" +
                "                   <geometry>\n" +
                "                       <mesh>\n" +
                f"                           <uri>model://circular_workspace_{radius}m.dae</uri>\n" +
                "                           <scale>1 1 1</scale>\n" +
                "                       </mesh>\n" +
                "                   </geometry>\n" +
                "               </visual>\n" +
                "           </link>\n" +
                "       </model>\n")
        
        for i in range(n):
            data += (f"       <actor name=\"agent{i+1}\">\n" +
                     f"          <pose>0 {i} 1.01 0 0 0</pose>\n" +
                     "          <skin>\n" +
                     "              <filename>stand.dae</filename>\n" +
                     "              <scale>1.0</scale>\n" +
                     "          </skin>\n" +
                     "          <animation name=\"walking\">\n" +
                     "              <filename>walk.dae</filename>\n" +
                     "              <scale>1.000000</scale>\n" +
                     "              <interpolate_x>true</interpolate_x>\n" +
                     "          </animation>\n" +
                     "       </actor>\n")
            
        data += ("   </world>\n" + "</sdf>")

        with open(f'{world_dir}{title}.world', 'w') as file:
            file.write(data)

def write_launch(title):
    data = ("import os\n" +
           "from launch import LaunchDescription\n" +
           "from launch.substitutions import LaunchConfiguration\n" +
           "from launch_ros.actions import Node\n" +
           "from launch.actions import ExecuteProcess\n" +
           "from ament_index_python.packages import get_package_share_directory\n" +
           "def generate_launch_description():\n" +
           f"    world_file_name = '{title}.world'\n" +
           "    pkg_dir = get_package_share_directory('world_sfm_hsfm_plugins')\n" +
           "    os.environ[\"GAZEBO_MODEL_PATH\"] = os.path.join(pkg_dir, 'models')\n" +
           "    world = os.path.join(pkg_dir, 'worlds', world_file_name)\n" +
           f"    config = os.path.join(pkg_dir,'config','{title}.yaml')\n" +
           "    agents_loader=Node(\n" +
           "        package = 'world_sfm_hsfm_plugins',\n" +
           "        name = 'agent_params_loader',\n" +
           "        executable = 'load_agent_params',\n" +
           "        parameters = [config])\n" +
           "    spawn_entity = Node(package='world_sfm_hsfm_plugins', executable='spawn_robot.py',\n" +
           "                        arguments=['pioneer3at', 'demo', '6.0', '0.0', '0.0'],\n" +
           "                        output='screen')\n" +
           "    gazebo = ExecuteProcess(\n" +
           "            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',\n" +
           "            '-s', 'libgazebo_ros_factory.so', '--pause'],\n" +
           "            output='screen')\n" +
           "    return LaunchDescription([\n" +
           "        spawn_entity,\n" +
           "        agents_loader,\n" +
           "        gazebo])")
    
    with open(f'{launch_dir}{title}.launch.py', 'w') as file:
            file.write(data)

def main():
    radius = int(input("Specify the desired radius of the circular workspace (3, 4, 5, 6, or 7):\n"))
    n_actors = int(input("Specify the number of actors to insert in the experiment:\n"))
    model = bool(int(input("Specify the model that guides human motion (0: SFM, 1: HSFM):\n")))

    ### COMPUTATIONS
    if (not model): model_title = "sfm"
    else: model_title = "hsfm"
    title = f"circular_op_{model_title}_{n_actors}_{radius}m"
    arch = (2 * math.pi) / (n_actors + 1)
    dist_center = radius - 0.5

    init_pos = []
    goal = []
    init_yaw = []

    for i in range(n_actors + 1):
        init_pos.append([round(dist_center * math.cos(arch * i),4), round(dist_center * math.sin(arch * i),4)])
        init_yaw.append(round(to_degree(bound_angle(-math.pi + arch * i)),4))
        goal.append([-init_pos[i][0],-init_pos[i][1]])

    ## GENERATE YAML
    write_yaml(radius, model, init_pos, init_yaw, goal, title)
    
    ## GENERATE WORLD FILE
    write_world(radius, model, n_actors, title)

    ## GENERATE LAUNCH FILE
    write_launch(title)

    print(f"Generation successfully completed! Build you ros2 workspace and launch with the following command: \n\n" +
          "ros2 launch world_sfm_hsfm_plugins circular_op_{model_title}_{n_actors}_{radius}m.launch.py\n")

if __name__ == "__main__":
    main()
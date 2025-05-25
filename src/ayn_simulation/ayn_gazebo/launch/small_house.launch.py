# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    world_file_name = 'small_house.world'
    package_dir = get_package_share_directory('ayn_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(
        get_package_share_directory('ayn_gazebo'),
        'worlds',
        'small_house.world'
    )

    gzclient_cmd = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    gzserver_cmd = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('ayn_gazebo'),
                     'models'))

    return LaunchDescription([
        set_env_vars_resources,
        # DeclareLaunchArgument(
        #   'world',
        #   default_value=[os.path.join(package_dir, 'worlds', world_file_name), ''],
        #   description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        # DeclareLaunchArgument(
        #     name='use_sim_time',
        #     default_value='true'
        # ),
        # DeclareLaunchArgument('state',
        #     default_value='true',
        #     description='Set "true" to load "libgazebo_ros_state.so"'),
        gzserver_cmd,
        gzclient_cmd,
    ])


if __name__ == '__main__':
    generate_launch_description()

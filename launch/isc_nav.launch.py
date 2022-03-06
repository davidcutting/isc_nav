# MIT License
#
# Copyright (c) 2022 David Cutting
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    get_package_share_directory('isc_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_frame = LaunchConfiguration('robot_frame', default='base_footprint')
    map_frame = LaunchConfiguration('map_frame', default='map')
    tf_timeout = LaunchConfiguration('tf_timeout', default='0.03')
    lookahead_distance = LaunchConfiguration('lookahead_distance', default='0.5')
    desired_linear_velocity = LaunchConfiguration('desired_linear_velocity', default='1.0')
    desired_angular_velocity = LaunchConfiguration('desired_angular_velocity', default='1.5')

    pure_pursuit = Node(
        package='isc_nav',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_frame': robot_frame,
            'tf_timeout': tf_timeout,
            'lookahead_distance': lookahead_distance,
            'desired_linear_velocity': desired_linear_velocity,
            'desired_angular_velocity': desired_angular_velocity,
        }])

    path_planner_node = Node(
        package='isc_nav',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_frame': robot_frame,
            'map_frame': map_frame,
        }])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Joy button which triggers a drive mode switch event'),

        # Nodes
        pure_pursuit,
        path_planner_node,
    ])

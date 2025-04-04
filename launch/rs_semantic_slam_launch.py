# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch file for semantic SLAM."""

# import os
from launch_ros.actions import Node
from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch semantic SLAM node."""
    return LaunchDescription([
        Node(
            package='as2_slam',
            executable='as2_slam_node',
            name='semantic_slam_node',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'pose_topic': '/drone0/self_localization/pose'},
                        {'detections_topic': '/drone0/processed_gate_poses_array'},
                        {'map_frame': 'drone0/map'},
                        {'odom_frame': 'drone0/odom'},
                        {'robot_frame': 'drone0/base_link'},
                        {'viz_main_markers_topic': 'slam_viz/markers/main'},
                        {'viz_temp_markers_topic': 'slam_viz/markers/temp'},
                        ],
            emulate_tty=True,
        ),
    ])

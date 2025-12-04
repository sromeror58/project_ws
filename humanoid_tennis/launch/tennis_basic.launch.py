"""Launch humanoid tennis arm demo.

   ros2 launch humanoid_tennis tennis_basic.launch.py

This should start
  1) RVIZ, ready to view the G1 robot
  2) The robot_state_publisher to broadcast the G1 model
  3) The tennis arm node to move the joints
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # RViz config (reuse the utils one you already have for URDFs)
    rvizcfg = os.path.join(pkgdir('utils'), 'rviz', 'viewurdfplus.rviz')

    # G1 URDF file (29 dof)
    g1_share = pkgdir('g1_description')
    urdf = os.path.join(g1_share, 'urdf', 'g1_29dof.urdf')

    # Load URDF XML text
    with open(urdf, 'r') as f:
        robot_description = f.read()

    ######################################################################
    # NODES

    # Robot state publisher
    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # RViz
    node_rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rvizcfg],
        on_exit=Shutdown(),
    )

    # Tennis arm joint controller
    # (make sure setup.py installs an entry point called "basic_swing_node"
    # or change executable to whatever you actually used.)
    node_tennis = Node(
        name='tennis_arm',
        package='humanoid_tennis',
        executable='swing_node', #change this if you change the node
        output='screen',
    )

    # Simple Simulation Node
    node_sim = Node(
        name='simple_sim',
        package='humanoid_tennis',
        executable='simple_sim_node',
        output='screen',
    )

    # Ball Spawner Node
    node_ball_spawner = Node(
        name='ball_spawner',
        package='ball_spawner',
        executable='ball_spawner_node',
        output='screen',
    )

    ######################################################################
    # RETURN
    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_tennis,
        node_sim,
        node_ball_spawner,
    ])

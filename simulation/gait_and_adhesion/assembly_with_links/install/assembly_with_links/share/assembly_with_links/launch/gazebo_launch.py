import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('assembly_with_links')
    urdf_path = os.path.join(pkg_share, 'urdf', 'assembly_with_links.urdf')
    config_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    world_path = os.path.join(pkg_share, 'worlds', 'crawler_hexapod.world')

    # Required for the six per-leg hexapod_link_attacher plugin instances
    # (declared in crawler_hexapod.world) to be found. Adjust this path if
    # your workspace for hexapod_link_attacher lives somewhere else.
    os.environ["GAZEBO_PLUGIN_PATH"] = (
        "/home/diego/hexapod_link_attacher_ws/install/hexapod_link_attacher/lib:"
        + os.environ.get("GAZEBO_PLUGIN_PATH", "")
    )

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    robot_description_content = re.sub(
        r'<\?xml[^?]*\?>\s*', '', robot_description_content, count=1
    )

    # Strip all XML comments before this URDF is used anywhere. This is a
    # required workaround, not cleanup: gazebo_ros2_control (Gazebo Classic
    # plugin, 0.4.7+) has a confirmed upstream bug where its parameter-
    # override parser fails on ANY colon inside an XML comment when it
    # re-serializes the URDF internally -- see
    # https://github.com/ros-controls/gazebo_ros2_control/issues/295. That
    # produces exactly the "parser error Couldn't parse parameter override
    # rule" seen in our launch log, which prevents controller_manager from
    # ever coming up. Comments accumulate easily in a URDF that's been
    # hand-edited a lot (explanatory notes, URLs, "e.g.:" etc. all contain
    # colons), so stripping them here -- once, centrally -- is more robust
    # than manually avoiding colons in every comment added from now on.
    robot_description_content = re.sub(
        r'<!--.*?-->', '', robot_description_content, flags=re.DOTALL
    )

    meshes_path = os.path.join(pkg_share, 'meshes')
    robot_description_content = robot_description_content.replace(
        'package://assembly_with_links/meshes/',
        f'file://{meshes_path}/'
    )
    
    robot_description_content = robot_description_content.replace(
        '__CONTROLLER_YAML_PATH__',
        config_path
    )

    tmp_urdf = '/tmp/assembly_with_links_resolved.urdf'
    with open(tmp_urdf, 'w') as f:
        f.write(robot_description_content)

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': 'true', 'pause': 'false', 'world': world_path}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-file', tmp_urdf,
            '-entity', 'assembly_with_links',
            # Spawns above the blade's top surface near its root (wide end,
            # ~0.99m chord there -- enough width for the robot to stand),
            # not at the world origin/flat ground like before. Z is the
            # computed top-surface height at this span position (~0.089m)
            # plus a generous drop clearance, matching the same "spawn
            # above and let gravity settle it" approach used throughout
            # this project rather than trying to place it exactly on
            # contact.
            # Blade mesh is scaled 2.5x in crawler_hexapod.world -- this
            # spawn point is scaled proportionally to land at the same
            # relative position (25% along the span, near the root).
            '-x', '2.5', '-y', '0.0', '-z', '0.722',
        ],
        output='screen'
    )

    # NOTE: an earlier version of this launch file called
    # /gazebo/set_model_configuration here as a best-effort way to spawn
    # already in a crouched stance. Confirmed removed from your gazebo_ros
    # build (ros2 service list shows nothing for it) -- it was a silent
    # 5-second no-op on every single launch. The standing pose is instead
    # reached the normal way: gait_control.py's own control loop publishes
    # the crouched stance continuously at 20 Hz from the moment it starts,
    # with no key press required, so the robot should settle into it within
    # a couple of seconds of joint_trajectory_controller becoming active --
    # no launch-file-level pose injection needed.

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # CHAIN EVENTS: Spawn Broadcaster ONLY after Robot Spawns
    spawn_jsb_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    # CHAIN EVENTS: Spawn Arm Controller ONLY after Broadcaster is ready
    spawn_arm_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_jsb_event,
        spawn_arm_event
    ])

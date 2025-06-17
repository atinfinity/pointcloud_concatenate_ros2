from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    static_transform_publisher1_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'cloud1'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
    )
    static_transform_publisher2_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher2',
        arguments=[
            '--x', '-0.05', '--y', '0.5', '--z', '0',
            '--yaw', '1.571', '--pitch', '0', '--roll', '0',
            '--frame-id', 'cloud1', '--child-frame-id', 'cloud2'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
    )
    pointcloud_concat_node = Node(
        package='pointcloud_concatenate_ros2',
        executable='pointcloud_concat_node',
        parameters=[{
            '/target_frame': 'map',
            '/clouds': 2,
            '/hz': 10.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[('cloud_in1', '/cloud1'),
                    ('cloud_in2', '/cloud2'),
                    ('cloud_out', '/fusion')]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(static_transform_publisher1_cmd)
    ld.add_action(static_transform_publisher2_cmd)
    ld.add_action(pointcloud_concat_node)

    return ld

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'ti_mmwaver_0'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '-0.05', '--y', '0.5', '--z', '0',
                '--yaw', '1.571', '--pitch', '0', '--roll', '0',
                '--frame-id', 'ti_mmwaver_0', '--child-frame-id', 'ti_mmwaver_1'
            ]
        ),
        Node(
            package="pointcloud_concatenate_ros2",
            executable="pointcloud_concat_node",
            parameters=[{
                "/target_frame": "map",
                "/clouds": 2,
                "/hz": 10.0
            }],
            remappings=[("cloud_in1","/ti_radar_0/ti_mmwave_0"),
                        ("cloud_in2","/ti_radar_1/ti_mmwave_1"),
                        ("cloud_out","fusion")]
        )
    ])
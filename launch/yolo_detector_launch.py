from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from math import pi

camera = {

    'tf_pose': [
        '--x', '0.465',
        '--y', '0.0',
        '--z', '0.36',
        '--roll', f'{0.0}',
        '--pitch', '-2.225',
        '--yaw', f'-{0.0}'
    ]
}

def generate_launch_description():

    launchArguments = [
        DeclareLaunchArgument('color_image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.0'),
    ]

    nodes = [
        Node( # --- TF Frame for Camera ---
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='omx',
            name='cam_tf',
            arguments=[
                '--frame-id', 'world', '--child-frame-id', 'camera_link',
                *camera['tf_pose'],
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='rpd_percepta',
            executable='yolo_object_detector',
            namespace='rpd',
            name='yolo_object_detector',
            parameters=[{
                'color_image_topic': LaunchConfiguration('color_image_topic'),
                'model': LaunchConfiguration('model'),
                'conf': LaunchConfiguration('conf')
            }]
        ),
        Node(
            package='rpd_percepta',
            executable='yolo_convert_to_3d',
            namespace='rpd',
            name='yolo_convert_to_3d',
            parameters=[{
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_image_topic': LaunchConfiguration('depth_image_topic')
            }]
        )
    ]

    return LaunchDescription([*launchArguments, *nodes])
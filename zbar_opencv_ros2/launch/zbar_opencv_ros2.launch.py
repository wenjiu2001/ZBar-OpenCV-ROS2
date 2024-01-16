from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    usb_cam_launch = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen'
    )

    zbar_opencv_launch = Node(
        package='zbar_opencv_ros2',
        executable='zbar_opencv_ros2',
        name='zbar_opencv_ros2',
        parameters=[{'image_topic': 'image_raw'}, {'barcode_topic': 'zbar_opencv_code'}],
        output='screen'
    )

    return LaunchDescription([
        usb_cam_launch,
        zbar_opencv_launch
    ])

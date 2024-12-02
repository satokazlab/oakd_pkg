from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パッケージのディレクトリを取得
    package_share_directory = get_package_share_directory('oakd_pkg')

    # モデルファイルと設定ファイルのパス
    model_path = os.path.join(package_share_directory, 'models', '/home/minashigo/ros2_ws/src/oakd_pkg/models/model2.blob')
    config_path = os.path.join(package_share_directory, 'config', '/home/minashigo/ros2_ws/src/oakd_pkg/json/model2.json')

    return LaunchDescription([
        Node(
            package='oakd_pkg',
            executable='image_topic_test',
            parameters=[
                {'model_path': model_path},
                {'config_path': config_path}
            ],
            output='screen'
        )
    ])

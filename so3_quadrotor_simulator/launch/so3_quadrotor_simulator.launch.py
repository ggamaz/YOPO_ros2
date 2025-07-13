
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取当前文件所在目录，用于 logger_file_name 参数
    # 注意：在ROS 2中，不推荐在launch文件中直接使用 find 命令查找包路径
    # 如果 so3_control 包的 logger 目录是用于存储日志的，你可能需要考虑一个固定的路径或者在运行时通过其他方式指定
    # 这里为了兼容性，我们假设可以通过 os.path.join 构造一个路径，但更好的做法是让节点自己处理日志路径
    # 或者通过参数传递一个更灵活的路径
    # 例如：logger_dir = os.path.join(os.getenv('HOME'), '.ros', 'so3_control_logs')
    # 对于本示例，我们将简化处理，假设 logger_file_name 是一个相对路径或节点能理解的路径
    # 如果 so3_control 真的需要找到自己的包目录，你可能需要修改 so3_control 节点来适应 ROS 2 的最佳实践
    so3_control_share_dir = os.path.join(os.getenv('COLCON_PREFIX_PATH').split(os.pathsep)[0], 'share', 'so3_control')
    logger_path = os.path.join(so3_control_share_dir, 'logger', '') # 确保以斜杠结尾

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'init_x',
            default_value='0.0',
            description='Initial X position of the quadrotor.'
        ),
        DeclareLaunchArgument(
            'init_y',
            default_value='0.0',
            description='Initial Y position of the quadrotor.'
        ),
        DeclareLaunchArgument(
            'init_z',
            default_value='2.0',
            description='Initial Z position of the quadrotor.'
        ),

        # quadrotor_simulator_so3 node
        Node(
            package='so3_quadrotor_simulator',
            executable='quadrotor_simulator_so3',
            name='quadrotor_simulator_so3',
            output='screen',
            parameters=[
                {'rate/odom': 50.0},
                {'simulator/init_state_x': LaunchConfiguration('init_x')},
                {'simulator/init_state_y': LaunchConfiguration('init_y')},
                {'simulator/init_state_z': LaunchConfiguration('init_z')},
            ],
            remappings=[
                ('~/odom', '/sim/odom'),
                ('~/imu', '/sim/imu'),
                ('~/cmd', 'so3_cmd'),
                ('~/force_disturbance', 'force_disturbance'),
                ('~/moment_disturbance', 'moment_disturbance'),
            ]
        ),

        # network_controller_node
        Node(
            package='so3_control',
            executable='network_control_node',
            name='network_controller_node',
            output='screen',
            parameters=[
                {'is_simulation': True},
                {'use_disturbance_observer': True},
                {'hover_thrust': 0.375},
                {'record_log': False},
                {'logger_file_name': logger_path}, # 使用构造的路径
            ],
            remappings=[
                ('~/odom', '/sim/odom'),
                ('~/imu', '/sim/imu'),
                ('~/position_cmd', '/so3_control/pos_cmd'),
                ('~/so3_cmd', 'so3_cmd'),
            ]
        ),
    ])
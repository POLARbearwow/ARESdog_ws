from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='串口波特率'
    )
    
    # 创建电机角度节点
    motor_angle_node = Node(
        package='ares_comm',
        executable='motor_angle_node',
        name='motor_angle',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'read_timeout_ms': 100,
            'motor_angle_data_id': 0x0003,  # 电机角度数据帧的ID
            'encoder_to_radians_factor': 0.01,  # 编码器值转换为弧度的系数 (double类型)
            'motor_count_per_frame': 10,    # 每帧包含的电机数量
            'debug_output': False,          # 是否输出调试信息
        }]
    )
    
    # 返回启动描述
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        motor_angle_node
    ]) 
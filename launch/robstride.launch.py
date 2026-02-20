import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    packages_name = 'robstride_hardware_interface'

    config_file_path = os.path.join(
        get_package_share_directory(packages_name),
        'config',
        'motor_setting.yaml'
    )

    # YAML에서 CAN 인터페이스 목록과 baud_rate 읽기
    with open(config_file_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config.get('hardware_interface_node', {}).get('ros__parameters', {})
    can_interfaces = params.get('can_interfaces', ['can0'])
    baud_rate = params.get('baud_rate', 1000000)

    # CAN 인터페이스 설정 명령 생성
    can_setup_actions = []
    for can_if in can_interfaces:
        can_setup_actions.append(
            LogInfo(msg=f'[CAN Setup] Configuring {can_if} with bitrate {baud_rate}...')
        )
        # down → bitrate 설정 + up (한 줄로 실행)
        can_setup_actions.append(
            ExecuteProcess(
                cmd=[
                    'sudo', 'bash', '-c',
                    f'ip link set {can_if} down 2>/dev/null; '
                    f'ip link set {can_if} up type can bitrate {baud_rate} && '
                    f'echo "[CAN Setup] {can_if} is UP (bitrate={baud_rate})" || '
                    f'echo "[CAN Setup] FAILED to configure {can_if}"'
                ],
                output='screen'
            )
        )

    # CAN 설정 후 1초 대기 후 노드 시작
    hardware_interface_node = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[CAN Setup] Starting hardware_interface_node...'),
            Node(
                package=packages_name,
                executable='hardware_interface_node',
                name='hardware_interface_node',
                output='screen',
                parameters=[config_file_path]
            )
        ]
    )

    return LaunchDescription(
        can_setup_actions + [hardware_interface_node]
    )

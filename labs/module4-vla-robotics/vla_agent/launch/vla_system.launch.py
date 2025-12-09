from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'openai_api_key',
            default_value='',
            description='OpenAI API key for LLM planning'
        ),

        # VLA Agent main node
        Node(
            package='vla_agent',
            executable='vla_agent',
            name='vla_agent',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'agent_name': 'humanoid_vla_agent'},
                {'enable_vision': True},
                {'enable_language': True},
                {'enable_action': True},
                {'agent_frequency': 10.0},
            ]
        ),

        # Voice command interface node
        Node(
            package='vla_agent',
            executable='voice_interface',
            name='voice_command_interface',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'sample_rate': 16000},
                {'recording_duration': 5.0},
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
            ]
        ),

        # LLM planning node
        Node(
            package='vla_agent',
            executable='llm_planner',
            name='llm_planning',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'openai_model': 'gpt-4-turbo'},
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
                {'max_retries': 3},
            ]
        ),

        # Action execution node
        Node(
            package='vla_agent',
            executable='action_executor',
            name='action_execution',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'max_linear_velocity': 0.5},
                {'max_angular_velocity': 1.0},
            ]
        )
    ])
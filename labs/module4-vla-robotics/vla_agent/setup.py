from setuptools import setup

package_name = 'vla_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Team',
    maintainer_email='humanoid-team@example.com',
    description='Vision-Language-Action agent for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_agent = vla_agent.vla_agent:main',
            'voice_interface = vla_agent.voice_command_interface:main',
            'local_voice = vla_agent.local_voice_recognition:main',
            'llm_planner = vla_agent.llm_planning:main',
            'action_executor = vla_agent.action_execution:main',
        ],
    },
)
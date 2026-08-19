from setuptools import find_packages, setup

package_name = 'my_offboard_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cre',
    maintainer_email='cre@todo.todo',
    description='TODO: Package description',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_example = my_offboard_ctrl.offboard_ctrl_example:main',
            'control_keyboard = my_offboard_ctrl.control_keyboard:main',
            'telemetry_pub = my_offboard_ctrl.drone_telemetry_publisher:main',
            'drone_controller = my_offboard_ctrl.drone_controller:main',
            'actions_executer = my_offboard_ctrl.actions_executer:main',
            'action_executer_attitudeThrust = my_offboard_ctrl.action_executer_attitudeThrust:main',
            'test_attitudeThrust = my_offboard_ctrl.test_attitudeThrust:main',
            'landing = my_offboard_ctrl.landing_PID:main',
            'test_vent = my_offboard_ctrl.test_vent:main',
            'action_train_speed = my_offboard_ctrl.action_train_speed:main',
            'action_speed = my_offboard_ctrl.action_speed:main',
        ],
    },
)

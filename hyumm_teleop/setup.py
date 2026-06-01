from setuptools import setup

package_name = 'hyumm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='fndlrkdl94@gmail.com',
    description='Keyboard teleop -> /cmd_vel for HYU-MM (ROS2).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard ='
            ' hyumm_teleop.teleop_twist_keyboard:main',
        ],
    },
)

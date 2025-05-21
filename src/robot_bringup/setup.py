from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name + '/launch',
            ['launch/control_launch.py']),
        # config files
        ('share/' + package_name + '/config',
            ['config/robot_controllers.yaml',
             'config/rvisconfig.rviz']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lac',
    maintainer_email='amhun@pdx.edu',
    description='Bringup for differential-drive robot via ros2_control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # no CLI tools yet
        ],
    },
)

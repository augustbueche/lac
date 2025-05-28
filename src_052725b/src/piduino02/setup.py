from setuptools import setup

package_name = 'piduino02'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='amhunpdx',
    maintainer_email='amhun@pdx.edu',
    description='ROS 2 package for Raspberry Pi to control an Arduino-based robot over serial',
    license='MIT',
    entry_points={
        'console_scripts': [
           'controller = piduino02.controller:main',
            'motortest = piduino02.motortest:main',
            'encoderwatch = piduino02.encoderwatch:main',
            'serial_publisher = piduino02.serial_publisher:main',
             'oled_display = piduino02.oled_display:main',
        ],
    },
)
 
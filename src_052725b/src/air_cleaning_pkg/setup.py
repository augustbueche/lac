from setuptools import find_packages, setup

package_name = 'air_cleaning_pkg'

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
    maintainer='august',
    maintainer_email='augustruizbueche@gmail.com',
    description='Air cleaning package : includes publisher node that prints pm data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pm_sensor_publisher_node = air_cleaning_pkg.pm_sensor_publisher_node:main"
        ],
    },
)

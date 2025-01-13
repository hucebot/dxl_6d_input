from setuptools import find_packages, setup

package_name = 'ros2_dxl_6d_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dxl_6d.launch.py']),
        ('share/' + package_name + '/config', ['config/dxl_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clemente Donoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='ROS 2 package for Dynamixel 6-DOF input control',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dxl_6d = ros2_dxl_6d_input.dxl_6d:main',
        ],
    },
)

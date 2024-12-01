from setuptools import find_packages, setup

package_name = 'ros2_blue_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        ('share/' + package_name + '/launch', ['launch/blue_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blue',
    maintainer_email='blue@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
  #          'publisher = ros2_blue_interface.publisher:main',
           # 'node_script = ros2_blue_interface.node_script:main',
 #           'publisher2 = ros2_blue_interface.publisher2:main',
  #          'interface_node = ros2_blue_interface.interface_node:main',
  #          'interface_joy = ros2_blue_interface.interface_joy:main',
            'joy_node = ros2_blue_interface.joy_node:main',
            'simple_joy_node = ros2_blue_interface.simple_joy_node:main',
            #'joy_linux_node = ros2_blue_interface.interface_joy:main',
   #         'user_node = ros2_blue_interface.user_node:main',
            'bluerov_node = ros2_blue_interface.bluerov_node:main',
            'simple_bluerov_node = ros2_blue_interface.simple_bluerov_node:main',
            'pubs_node = ros2_blue_interface.pubs:main',
            'subs_node = ros2_blue_interface.subs:main',
            'bridge_node = ros2_blue_interface.bridge_node:main',
            'server_node = ros2_blue_interface.server_node:main',
            'client_node = ros2_blue_interface.client_node:main',
            'node_a = ros2_blue_interface.node_a:main',
            'node_b = ros2_blue_interface.node_b:main',
            'fake_gps = ros2_blue_interface.fake_gps_node:main',
    #        'topic_debug = ros2_blue_interface.topic_debug:main',
        
        ],
    },
)


#joy_linux_node
# 'interface_joy = ros2_blue_interface.interface_joy:main',
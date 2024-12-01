from setuptools import setup

package_name = 'cola2_stonefish'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=["cola2_stonefish.logitechF310teleop","cola2_stonefish.bluerov2_logitechF310teleop.py"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bluerov2_heavy_simulation.launch.py', 'launch/bluerov_fls_simulation.launch.py','launch/blueboat_launch.py','launch/falcon_launch.py', 'launch/bluerov2_laser_simulation.launch.py', 'launch/bluerov2_alpha.py', 'launch/console_test.py', 'launch/asv_auv_sim.py' ]),
        ('lib/' + package_name, [
            'scripts/logitechF310teleop.py',
            'scripts/magpy_subsea_cable.py',
            'scripts,bluerov2_logitechF310teleop.py',
            'scripts/odom2tf.py',
            'scripts/openmeteo.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Grimaldi',
    maintainer_email='michelegrmld@egmail.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cola2_stonefish_node = cola2_stonefish.cola2_stonefish_node:main',
            'logitechF310teleop = cola2_stonefish.logitechF310teleop:main',
            'bluerov2_logitechF310teleop = cola2_stonefish.bluerov2_logitechF310teleop:main',
            'magpy_subsea_cable = cola2_stonefish.magpy_subsea_cable:main',
            'odom2tf = cola2_stonefish.odom2tf:main',
            'openmeteo = cola2_stonefish.openmeteo:main',
        ],
    },
)


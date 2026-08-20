from setuptools import find_packages, setup

package_name = 'honrobo_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/map', [
            'map/map.yaml', 'map/map.png',
            'map/map_red.yaml', 'map/map_red.png',
            'map/map_blue.yaml', 'map/map_blue.png',
            'map/map_test.yaml', 'map/map_test.png'
        ]),
    ],
    install_requires=[
        'setuptools',
        'python-can',
        'rclpy',
        'pygame',
        'pyserial',
        'websockets',
    ],
    zip_safe=True,
    maintainer='haru',
    maintainer_email='robotic.engineer.dream@gmail.com',
    description='honrobo_2026 ロボット制御パッケージ',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zikoiti_node  = honrobo_pkg.zikoiti_node:main',
            'can_node      = honrobo_pkg.can_node:main',
            'ps4_node      = honrobo_pkg.ps4_node:main',
            'roboware_node = honrobo_pkg.roboware_node:main',
            'web_node      = honrobo_pkg.web_node:main',
            'monitor_node  = honrobo_pkg.monitor_node:main',
        ],
    },
)

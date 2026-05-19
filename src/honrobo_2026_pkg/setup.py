from setuptools import find_packages, setup

package_name = 'honrobo_2026_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'python-can',
        'rclpy',
        'pygame',
    ],
    zip_safe=True,
    maintainer='saitoharuhi',
    maintainer_email='robotic.engineer.dream@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'can_node = honrobo_2026_pkg.can_node:main',
            'all_senser = honrobo_2026_pkg.all_senser:main',
            'all_senser_ukf = honrobo_2026_pkg.all_senser_ukf:main',
            'ps4_node = honrobo_2026_pkg.ps4_node:main',
            'roboware_node = honrobo_2026_pkg.roboware_node:main',
            'web_nav_node = honrobo_2026_pkg.web_nav_node:main',
            'camera_node = honrobo_2026_pkg.camera_node:main'
        ],
    },
)

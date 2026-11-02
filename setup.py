from setuptools import find_packages, setup

package_name = 'ekf_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Shrimed Sangani',
    maintainer_email='shrimed.sangani@gmail.com',
    description=(
        'Extended Kalman Filter localization for TurtleBot3 using '
        'ROS 2, noisy odometry, and IMU measurements.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = ekf_localization.ekf_node:main',
            'wrapper = ekf_localization.synthetic_noise_wrapper:main',
        ],
    },
)

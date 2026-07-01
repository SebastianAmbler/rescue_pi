from setuptools import setup

package_name = 'camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='claudecode67420@gmail.com',
    description='MJPEG HTTP bridge for a ROS CompressedImage topic.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mjpeg_bridge = camera_bridge.mjpeg_bridge:main',
        ],
    },
)

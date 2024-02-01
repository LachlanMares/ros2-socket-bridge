from setuptools import setup

package_name = 'ros2_socket_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lachlanmares',
    maintainer_email='lachlan.mares@adelaide.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_message_publisher = ros2_socket_bridge.multi_message_publisher:main',
            'multi_message_subscriber = ros2_socket_bridge.multi_message_subscriber:main',
            'multi_message_server = ros2_socket_bridge.multi_message_server:main',
            'multi_message_client = ros2_socket_bridge.multi_message_client:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'turtle_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_tf2_launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nsu',
    maintainer_email='nsu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = turtle_tf2.turtle_tf2_broadcaster:main',
            'carrot_tf2_broadcaster = turtle_tf2.carrot_tf2_broadcaster:main',
            'turtle_tf2_listener = turtle_tf2.turtle_tf2_listener:main',
        ],
    },
)

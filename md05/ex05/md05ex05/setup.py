from setuptools import find_packages, setup

package_name = 'md05ex05'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/random_movement.launch.py']),
        ('share/' + package_name, ['robot.gazebo.xacro']),
        ('share/' + package_name, ['robot.rviz'])
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
            'random_movement = md05ex05.random_movement:main',
        ],
    },
)

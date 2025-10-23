from setuptools import find_packages, setup

package_name = 'md04ex03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/follow_with_delay_launch.py']),
        ('share/' + package_name, ['package.xml']),
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
            'turtle_tf2_broadcaster = md04ex03.turtle_tf2_broadcaster:main',
            'delayed_tf_listener = md04ex03.delayed_tf_listener:main',
        ],
    },
)

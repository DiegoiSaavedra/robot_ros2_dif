from setuptools import setup

package_name = 'robot_difer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_difer_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Usuario',
    maintainer_email='usuario@example.com',
    description='Nodo base_driver para robot diferencial que publica odometría y transformaciones TF.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robot_difer.base_driver:main',
        ],
    },
)

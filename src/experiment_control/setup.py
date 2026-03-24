from setuptools import find_packages, setup

package_name = 'experiment_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/supply_valve_feature.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kklab',
    maintainer_email='kklab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'supply_valve_feature = experiment_control.supply_valve_feature:main',
            'pos_controller = cylinder_exp.pos_controller:main',
        ],
    },
)

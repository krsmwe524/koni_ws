from setuptools import setup
package_name = 'py_force_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koni',
    maintainer_email='example@example.com',
    description='Force (load) PI controller for PAM bench',
    license='MIT',
    entry_points={
        'console_scripts': [
            'force_controller = py_force_controller.force_controller:main',
            'single_loop_force_control = py_force_controller.single_force:main',
            'variable_force_controller = py_force_controller.variable_force:main',
            'prbs_generator = py_force_controller.prbs_generator:main',
            'experiment_logger = py_force_controller.experiment_logger:main',
        ],
    },
)
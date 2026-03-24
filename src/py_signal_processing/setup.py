from setuptools import setup

package_name = 'py_signal_processing'

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
    maintainer='kklab',
    maintainer_email='kklab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'analog_voltage_interpreter = py_signal_processing.analog_voltage_interpreter:main',
            'analog_voltage_interpreter_cyl = py_signal_processing.analog_voltage_interpreter_cyl:main',
            'analog_voltage_interpreter_koni = py_signal_processing.analog_voltage_interpreter_koni:main',
            'calculator = py_signal_processing.calculator:main',
            'calculator_angle = py_signal_processing.calculator_angle2:main',
            'calculator_force = py_signal_processing.calculator_force:main',
            'calculator_pressure = py_signal_processing.calculator_pressure:main',
            'calculator_pressure_map = py_signal_processing.calculator_pressure_map:main',

        ],
    },
)

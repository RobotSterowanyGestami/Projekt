from setuptools import setup

package_name = 'hal_ultrasonic_sensors'

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
    maintainer='projekt',
    maintainer_email='robotsterowanygestami@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hal_ultrasonic_sensor_driver = hal_ultrasonic_sensors.hal_ultrasonic_sensor_driver:main'
        ],
    },
)
from setuptools import find_packages, setup

package_name = 'mujoco_drone_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Rauch',
    maintainer_email='Rauch.Christian@gmx.de',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_sim = mujoco_drone_simulator.drone_sim:main'
        ],
    },
)

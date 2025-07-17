from setuptools import find_packages, setup

package_name = 'Carlim_Drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drive.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='limdoyeon',
    maintainer_email='limdoyeon@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering_node = Carlim_Drive.steering_node:main',
            'inwheel_node = Carlim_Drive.inwheel_node:main',
        ],
    },
)

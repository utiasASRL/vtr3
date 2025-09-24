from setuptools import find_packages, setup

package_name = 'vtr_multi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            # Explicitly install all launch files
            ('share/' + package_name + '/launch', [
                'launch/convoy_manager.launch.py',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asrl',
    maintainer_email='l.antonyshyn@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convoy_manager = vtr_multi_robot.convoy_manager:main',
        ],
    },
)

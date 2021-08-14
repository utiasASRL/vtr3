from setuptools import setup

package_name = 'vtr_ui'

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
    maintainer='Yuchen Wu',
    maintainer_email='cheney.wu@mail.utoronto.ca',
    description='VTR web-based GUI.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = vtr_ui.frontend.web_server:main',
            'socket_server = vtr_ui.frontend.socket_server:main',
            'socket_client = vtr_ui.socket_client.socket_client_node:main',
        ],
    },
)

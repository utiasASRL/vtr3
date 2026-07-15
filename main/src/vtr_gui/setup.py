import sys

from setuptools import setup
import os
import subprocess

from setuptools import setup
from setuptools.command.develop import develop
from setuptools.command.install import install

def npm_build_vtr_ui():
    try:
        result = subprocess.run(['npm', 'run', 'build'],
            check=True, 
            capture_output=True, 
            text=True,
            cwd=os.getenv("VTRUI")
        )
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"npm run build failed with code {e.returncode}", file=sys.stderr)
        print(e.stderr, file=sys.stderr)
        print(r"Likely you have not installed the npm packages. Run: npm --prefix ${VTRUI} install ${VTRUI}", file=sys.stderr)
        sys.exit(e.returncode)

class NPMBuildDevelop(develop):
    def run(self):
        npm_build_vtr_ui()
        super().run()

class NPMBuildInstall(install):
    def run(self):
        npm_build_vtr_ui()
        super().run()


package_name = 'vtr_gui'

def package_files(directory):
    data_files = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Construct relative path to maintain structure in install
            data_files.append((os.path.join('share', package_name, path), 
                                [os.path.join(path, filename)]))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_files('vtr_gui/vtr-gui/build')
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
            'setup_server = vtr_gui.setup_server:main',
            'web_server = vtr_gui.web_server:main',
            'socket_server = vtr_gui.socket_server:main',
            'socket_client = vtr_gui.socket_client:main',
            'multi_robot_web_server = vtr_gui.multi_robot_web_server:main',
            'multi_robot_socket_server = vtr_gui.multi_robot_socket_server:main',
            'multi_robot_socket_client = vtr_gui.multi_robot_socket_client:main',
            'setup_client = vtr_gui.setup_client:main',
        ],
    },
    cmdclass={
        'develop': NPMBuildDevelop, # Runs during 'colcon build --symlink-install'
        'install': NPMBuildInstall, # Runs during standard 'colcon build'
    },
)

from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'piper'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装其他文件，如参数文件
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_single_ctrl = piper.piper_ctrl_single_node:main',
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'buscabot_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='buscabot',
    maintainer_email='buscabot@todo.todo',
    description='Nodo de comunicación serial con ESP32',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'esp32_serial = buscabot_serial.esp32_serial:main',
        ],
    },
)

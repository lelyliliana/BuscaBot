from setuptools import setup

package_name = 'buscabot_argus_camera'

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
    maintainer='buscabot',
    maintainer_email='buscabot@todo.todo',
    description='Nodo de cámara para IMX219 usando nvarguscamerasrc en Jetson',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'argus_camera = buscabot_argus_camera.argus_camera_node:main',
        ],
    },
)

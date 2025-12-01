from setuptools import find_packages, setup

package_name = 'buscabot_vision'

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
    maintainer='buscabot',
    maintainer_email='buscabot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
    'face_detector = buscabot_vision.face_detector:main',
    'face_embedder_node = buscabot_vision.face_embedder_node:main',
    'face_enroller_node = buscabot_vision.face_enroller_node:main',
    'jetson_csi_node = buscabot_vision.jetson_csi_node:main',
    'face_target_tracker_node = buscabot_vision.face_target_tracker_node:main',
    'web_target_face = buscabot_vision.web_target_face:main',
    'csi_camera_node = buscabot_vision.csi_camera_node:main',
    'face_recognition_node = buscabot_vision.face_recognition_node:main',
],

},
)

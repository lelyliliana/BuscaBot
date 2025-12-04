BuscaBot  
Robot terrestre autónomo para búsqueda y rescate en terrenos difíciles  

Descripción general
-------------------
BuscaBot es un robot terrestre autónomo diseñado para la localización de personas en situaciones de emergencia. 
El proyecto integra robótica, visión por computador, inteligencia artificial y hardware especializado.

La plataforma está basada en:
- NVIDIA Jetson Orin Nano Super Developer Kit
- ROS2 Humble
- Cámara IMX219 con Argus Camera API
- Sensores y motores Dynamixel AX-12A
- RPLIDAR A1M8
- Impresión 3D con Bambu Lab X1-Carbon

Objetivo principal
------------------
Desarrollar un robot capaz de:
- Capturar video y procesarlo mediante visión por computador.
- Detectar rostros para apoyar tareas de búsqueda de personas.
- Desplazarse utilizando actuadores Dynamixel AX-12A.
- Realizar mapeo con sensores LIDAR.
- Implementar futuras estrategias de navegación autónoma.

Estructura del repositorio
--------------------------
BuscaBot/
│
├── src/
│   ├── buscabot_camera/          # Nodo ROS2 para captura con Argus Camera API
│   ├── buscabot_vision/          # Nodo ROS2 para detección de rostros
│   ├── buscabot_serial/          # Comunicación con actuadores y microcontroladores
│   ├── buscabot_argus_camera/    # Versión optimizada de captura NVMM
│   ├── DynamixelSDK/             # SDK oficial para motores AX-12A
│   └── rplidar_ros/              # Nodo ROS2 del sensor LIDAR A1M8
│
├── docs/
│   ├── InformeTecnico.pdf        # (se añadirá posteriormente)
│   ├── imagenes/
│   └── configuracion/
│
├── 3d_models/
│   ├── STL/
│   └── Blender/
│
├── LICENSE
├── README.md
└── .gitignore

Instalación
-----------
mkdir -p ~/buscabot_ws/src
cd ~/buscabot_ws
colcon build
source install/setup.bash

Nodos principales
-----------------
ros2 run buscabot_argus_camera argus_camera_node
ros2 run buscabot_vision face_detector --ros-args -p show_window:=false
ros2 launch rplidar_ros rplidar.launch.py
ros2 run buscabot_serial dynamixel_controller

Autoría
-------
Leli Liliana Díaz Izquierdo
Ingeniera de Sistemas – Uniremington

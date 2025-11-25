#BuscaBot
#Robot terrestre autónomo para búsqueda de personas en terrenos difíciles

#Descripción general

BuscaBot es un robot terrestre autónomo diseñado para la búsqueda de personas en escenarios de emergencia. El proyecto integra robótica, visión por computador, inteligencia artificial y hardware avanzado.

La plataforma está basada en:

- NVIDIA Jetson Orin Nano Super Developer Kit

- ROS2 Humble

- Cámara IMX219 con Argus Camera API

- Sensores y motores Dynamixel AX-12A

- RPLIDAR A1M8

- Impresión 3D con Bambu Lab X1-Carbon

El repositorio contiene el código fuente, nodos ROS2, archivos de diseño, documentación técnica y material de soporte.

#Objetivo principal

Desarrollar un robot capaz de:

- Capturar video y procesarlo mediante visión por computador.

- Detectar rostros para apoyar tareas de localización de personas.

- Desplazarse utilizando actuadores Dynamixel AX-12A.

- Realizar mapeo con sensores LIDAR.

- Implementar futuras estrategias de navegación autónoma.

#Estructura del repositorio

BuscaBot/
│
├── src/
│ ├── buscabot_camera/ (Nodo ROS2 para captura con Argus Camera API)
│ ├── buscabot_vision/ (Nodo ROS2 para detección de rostros)
│ ├── buscabot_serial/ (Comunicación con actuadores y microcontroladores)
│ ├── buscabot_argus_camera/ (Versión optimizada de captura NVMM)
│ ├── DynamixelSDK/ (SDK oficial para motores AX-12A)
│ └── rplidar_ros/ (Nodo del LIDAR A1M8)
│
├── docs/
│ ├── InformeTecnico.pdf (se añadirá posteriormente)
│ ├── imagenes/
│ └── configuracion/
│
├── 3d_models/
│ ├── STL/
│ └── Blender/
│
├── LICENSE
├── README.md
└── .gitignore

#Instalación del entorno en Jetson Orin Nano

Requisitos:

- Ubuntu 22.04 LTS

- JetPack 6

- ROS2 Humble instalado correctamente

- GStreamer con soporte Argus (nvarguscamerasrc)

- Miniconda (opcional)

#Preparación del workspace:

mkdir -p ~/buscabot_ws/src
cd ~/buscabot_ws
colcon build
source install/setup.bash

#Ejecución de nodos principales

- Nodo de cámara Argus (captura NVMM optimizada)

ros2 run buscabot_argus_camera argus_camera_node

- Nodo de visión (detección de rostros)

ros2 run buscabot_vision face_detector --ros-args -p show_window:=false

Este nodo publica en el tópico:
/faces (tipo std_msgs/String)

Ejemplo de mensaje publicado:
N_FACES=2; [(x1, y1, w1, h1), (x2, y2, w2, h2)]

- Nodo del LIDAR

ros2 launch rplidar_ros rplidar.launch.py

- Control Dynamixel

ros2 run buscabot_serial dynamixel_controller

#Estado actual del proyecto

- Integración estable del Argus Camera API con ROS2.

- Nodo de detección de rostros funcionando correctamente.

- Workspace limpio, estructurado y versionado.

- Nodos de LIDAR y Dynamixel integrados.

- En proceso: documentación técnica, modelado 3D y navegación autónoma.

#Próximos pasos

- Integración de IA para navegación autónoma.

- Movimiento avanzado del cuadrúpedo o plataforma móvil.

- Pruebas de campo.

- Implementación de reconocimiento avanzado en Jetson Orin Nano.

#Autoría

Proyecto desarrollado por:
Leli Liliana Díaz Izquierdo
Ingeniera de Sistemas – Uniremington
Ingeniera de Sistemas – Uniremington

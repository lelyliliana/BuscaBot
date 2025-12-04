<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8" />
  <title>BuscaBot – Módulo de Visión y Reconocimiento Facial</title>
</head>
<body>

  <h1>BuscaBot – Módulo de Visión y Reconocimiento Facial</h1>
  <p>
    <strong>Robot terrestre autónomo para operaciones de búsqueda y rescate</strong><br/>
    <strong>Desarrollado por:</strong> Leli Liliana Díaz Izquierdo – Uniremington · Versión Jetson Orin Nano
  </p>

  <hr/>

  <h2>📌 Descripción General</h2>
  <p>
    Este módulo implementa el sistema de <strong>visión por computadora</strong>,
    <strong>detección de rostros</strong>, <strong>extracción de embeddings faciales</strong> y
    <strong>seguimiento de objetivo</strong> del robot BuscaBot, utilizando una Jetson Orin Nano,
    ROS2 Humble, InsightFace y una cámara IMX219/IMX477.
  </p>
  <p>Incluye:</p>
  <ul>
    <li>Nodo ROS2 para publicar imágenes desde la cámara.</li>
    <li>Nodo de reconocimiento facial basado en embeddings de 512 dimensiones.</li>
    <li>Nodo de seguimiento de objetivo.</li>
    <li>Interfaz web para cargar la persona objetivo.</li>
    <li>Recarga dinámica del objetivo sin reiniciar ROS2.</li>
  </ul>
  <p>
    El sistema permite que el robot identifique en tiempo real si la persona que está frente a la cámara
    coincide con la identidad proporcionada mediante una fotografía cargada desde una interfaz web.
  </p>

  <h2>📁 Estructura del Proyecto</h2>
  <pre><code>buscabot_ws/
│
├── src/
│   ├── buscabot_vision/
│   │   ├── face_recognition_node.py
│   │   ├── face_target_tracker_node.py
│   │   ├── jetson_csi_node.py
│   │   ├── face_enroller_node.py
│   │   ├── face_embedder_node.py
│   │   ├── face_detector.py
│   │   └── ...
│
├── web/
│   └── face_web.py        &larr; Servidor Flask (interfaz web)
│
└── ~/.buscabot_face_gallery/
    ├── target_current.json   &larr; Embedding + nombre objetivo
    └── face_status.txt       &larr; Estado actual del reconocimiento
</code></pre>

  <h2>🚀 Requisitos</h2>

  <h3>Hardware</h3>
  <ul>
    <li>Jetson Orin Nano.</li>
    <li>Cámara IMX219 y/o IMX477 CSI.</li>
    <li>Almacenamiento suficiente para modelos e imágenes.</li>
  </ul>

  <h3>Software</h3>
  <ul>
    <li>Ubuntu para Jetson.</li>
    <li>ROS2 Humble.</li>
    <li>Python 3.10.</li>
    <li>Entorno conda (para InsightFace y Flask).</li>
    <li>ONNX Runtime.</li>
    <li>OpenCV.</li>
    <li>InsightFace.</li>
  </ul>

  <h2>🟣 Instalación y Configuración</h2>

  <h3>1. Crear entorno conda</h3>
  <pre><code>conda create -n buscabot python=3.10
conda activate buscabot
pip install flask opencv-python insightface onnxruntime
</code></pre>

  <h3>2. Compilar el workspace ROS2</h3>
  <pre><code>cd ~/buscabot_ws
colcon build
source install/setup.bash
</code></pre>

  <h2>🟣 Cómo Ejecutar el Sistema Completo</h2>
  <p>BuscaBot usa <strong>3 terminales</strong> (consolas) para el módulo de visión.</p>

  <h3>🔵 Consola 1 – Cámara (SIN conda)</h3>
  <pre><code>cd ~/buscabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run buscabot_vision jetson_csi_node
</code></pre>
  <p>Este nodo publica imágenes en el tópico <code>/image_raw</code>.</p>

  <h3>🔴 Consola 2 – Reconocimiento Facial (CON conda)</h3>
  <pre><code>conda activate buscabot
cd ~/buscabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run buscabot_vision face_recognition_node
</code></pre>
  <p>Funciones principales:</p>
  <ul>
    <li>Detecta rostros con InsightFace.</li>
    <li>Genera y compara embeddings de 512 dimensiones.</li>
    <li>Publica resultados en <code>/face_recognition</code>.</li>
    <li>Escribe el estado en <code>~/.buscabot_face_gallery/face_status.txt</code>.</li>
  </ul>

  <h3>🟢 Consola 3 – Interfaz Web (CON conda)</h3>
  <pre><code>conda activate buscabot
cd ~/buscabot_ws/web
python face_web.py
</code></pre>
  <p>
    Acceso desde navegador:<br/>
    <code>http://&lt;IP_JETSON&gt;:8000</code>
  </p>

  <h2>🧠 Flujo del Reconocimiento Facial</h2>
  <ol>
    <li>La cámara publica imágenes en <code>/image_raw</code>.</li>
    <li>El nodo <code>face_recognition_node</code> recibe cada frame.</li>
    <li>Se detectan rostros y se obtienen embeddings mediante InsightFace.</li>
    <li>
      El embedding detectado se compara con el embedding objetivo almacenado
      (512 dimensiones, vector normalizado).
    </li>
    <li>
      Si la similitud es mayor o igual al umbral configurado:
      se considera que la persona objetivo ha sido detectada.
    </li>
    <li>
      El resultado se publica en <code>/face_recognition</code> y se registra en
      <code>face_status.txt</code>, que es leído por la interfaz web.
    </li>
  </ol>

  <h2>🟣 Recarga Dinámica del Objetivo</h2>
  <p>
    Cuando el usuario sube una fotografía en la interfaz web, se genera o actualiza el archivo:
  </p>
  <pre><code>~/.buscabot_face_gallery/target_current.json</code></pre>
  <p>Este archivo contiene:</p>
  <ul>
    <li>El nombre de la persona objetivo (<code>person_name</code>).</li>
    <li>El embedding facial normalizado (vector de 512 dimensiones).</li>
  </ul>
  <p>
    El nodo <code>face_recognition_node</code> monitorea la fecha de modificación
    (<em>mtime</em>) de <code>target_current.json</code>. Cada vez que llega un nuevo frame:
  </p>
  <ul>
    <li>Verifica si el archivo ha cambiado.</li>
    <li>Si cambió, recarga el nombre y el embedding objetivo en memoria.</li>
    <li>Registra en <code>face_status.txt</code> un mensaje indicando que se ha cargado un nuevo objetivo.</li>
    <li>Continúa el reconocimiento facial usando el nuevo embedding, sin reiniciar ROS2.</li>
  </ul>
  <p>
    Esto permite cambiar la persona objetivo de forma dinámica, manteniendo el sistema en operación
    continua y evitando tiempos muertos por reinicios.
  </p>

  <h2>📡 Tópicos ROS2</h2>

  <table border="1" cellpadding="6" cellspacing="0">
    <thead>
      <tr>
        <th>Tópico</th>
        <th>Tipo</th>
        <th>Descripción</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><code>/image_raw</code></td>
        <td><code>sensor_msgs/Image</code></td>
        <td>Frames de la cámara (entrada del sistema de visión).</td>
      </tr>
      <tr>
        <td><code>/face_recognition</code></td>
        <td><code>std_msgs/String</code></td>
        <td>Resultado textual del reconocimiento (detección o no coincidencia).</td>
      </tr>
    </tbody>
  </table>

  <h2>📜 Licencia</h2>
  <p>
    Software desarrollado por <strong>Leli Liliana Díaz Izquierdo</strong>.<br/>
    Proyecto académico y de investigación – Uniremington, 2025.
  </p>

  <h2>🧩 Notas Finales</h2>
  <ul>
    <li>
      Este módulo integra IA, visión por computadora y ROS2 sobre una plataforma Jetson Orin Nano.
    </li>
    <li>
      La arquitectura está diseñada para operar en tiempo real, con soporte para cambios dinámicos
      en la persona objetivo.
    </li>
    <li>
      La solución es modular, extensible y adecuada para escenarios de búsqueda y rescate en terrenos complejos.
    </li>
  </ul>

</body>
</html>


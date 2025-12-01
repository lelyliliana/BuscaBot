import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import insightface
from insightface.app import FaceAnalysis
import os
import json
from datetime import datetime

GALLERY_DIR = os.path.expanduser("~/.buscabot_face_gallery")
os.makedirs(GALLERY_DIR, exist_ok=True)

class FaceEnrollerNode(Node):
    def __init__(self):
        super().__init__('face_enroller_node')

        # Parámetros
        self.declare_parameter('person_name', 'persona_desconocida')
        self.person_name = (
            self.get_parameter('person_name').get_parameter_value().string_value
        )

        self.bridge = CvBridge()

        # 👉 flag para guardar solo un frame de depuración
        self.debug_saved = False

        # Cargar modelo InsightFace
        self.get_logger().info(f'🧑‍💻 Empezando enrolamiento para: {self.person_name}')
        self.get_logger().info('📦 Cargando modelo InsightFace (buffalo_l) en CPU...')
        self.app = FaceAnalysis(
            name='buffalo_l',
            root=os.path.expanduser('~/.insightface'),
            providers=['CPUExecutionProvider']
        )
        # det_size 640x640
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.get_logger().info('✅ Modelo InsightFace cargado.')

        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Acumulación de embeddings
        self.embeddings = []
        self.min_frames = 10  # nº de muestras para tener embedding robusto

    def image_callback(self, msg: Image):
        # Convertir a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 👉 Guardar un frame de depuración SOLO la primera vez
        if not self.debug_saved:
            debug_path = os.path.expanduser('~/debug_frame_enroller.jpg')
            try:
                import cv2
                cv2.imwrite(debug_path, frame)
                self.get_logger().info(
                    f'🖼 Frame de depuración guardado en: {debug_path}'
                )
            except Exception as e:
                self.get_logger().warn(f'No se pudo guardar debug_frame: {e}')
            self.debug_saved = True

        # Ejecutar detector
        faces = self.app.get(frame)

        if len(faces) == 0:
            self.get_logger().info('🙂 No hay caras detectadas en el frame')
            return

        self.get_logger().info(f'😎 {len(faces)} cara(s) detectadas en este frame.')

        # Tomamos solo la cara más grande (por si hay varias)
        faces_sorted = sorted(faces, key=lambda f: f.bbox[2] * f.bbox[3], reverse=True)
        main_face = faces_sorted[0]

        embedding = main_face.embedding
        self.embeddings.append(embedding.tolist())  # convertir a lista para JSON

        self.get_logger().info(
            f'✅ Muestra {len(self.embeddings)}/{self.min_frames} capturada para {self.person_name}'
        )

        if len(self.embeddings) >= self.min_frames:
            self.save_embedding_and_shutdown()

    def save_embedding_and_shutdown(self):
        # Promedio de embeddings
        emb_array = np.array(self.embeddings, dtype=np.float32)
        emb_mean = emb_array.mean(axis=0)

        data = {
            "person_name": self.person_name,
            "created_at": datetime.now().isoformat(),
            "embedding": emb_mean.tolist()
        }

        filename = os.path.join(GALLERY_DIR, f"{self.person_name}.json")
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(
            f'💾 Embedding guardado en: {filename}'
        )
        self.get_logger().info('🎉 Enrolamiento completado. Cerrando nodo.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FaceEnrollerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

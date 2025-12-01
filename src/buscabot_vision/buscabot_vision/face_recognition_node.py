import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from insightface.app import FaceAnalysis


GALLERY_DIR = Path.home() / ".buscabot_face_gallery"


def load_target_embedding(name: str) -> np.ndarray:
    json_path = GALLERY_DIR / f"{name}.json"
    data = json.loads(json_path.read_text())
    emb = np.array(data["embedding"], dtype=np.float32)

    # Normalizar el embedding (esto es CLAVE)
    norm = np.linalg.norm(emb)
    if norm == 0:
        raise RuntimeError(f"Embedding de {name} tiene norma 0")
    emb = emb / norm
    return emb


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__("face_recognition_node")

        # Parámetros
        self.declare_parameter("target_name", "Leli_Liliana")
        self.declare_parameter("similarity_threshold", 0.5)

        self.target_name = (
            self.get_parameter("target_name")
            .get_parameter_value()
            .string_value
        )
        self.threshold = (
            self.get_parameter("similarity_threshold")
            .get_parameter_value()
            .double_value
        )

        # Cargar embedding objetivo
        self.get_logger().info(f"Cargando embedding para: {self.target_name}")
        self.target_emb = load_target_embedding(self.target_name)
        self.get_logger().info(
            f"Embedding cargado. Norma = {np.linalg.norm(self.target_emb):.3f}"
        )

        # Inicializar InsightFace
        self.get_logger().info("Inicializando modelo InsightFace (buffalo_l)...")
        self.app = FaceAnalysis(name="buffalo_l")
        self.app.prepare(ctx_id=0, det_size=(640, 480))
        self.get_logger().info("Modelo InsightFace listo.")

        # ROS2: bridge, sub y pub
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10,
        )

        self.pub_result = self.create_publisher(String, "/face_recognition", 10)

        self.get_logger().info(
            f"✅ Nodo FaceRecognitionNode inicializado. "
            f"Objetivo: {self.target_name}, umbral: {self.threshold}"
        )

    def image_callback(self, msg: Image):
        # Convertir a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detectar rostros
        faces = self.app.get(frame)
        if not faces:
            return

        best_sim = -1.0
        for face in faces:
            emb = face.normed_embedding  # ya viene normalizado
            sim = float(np.dot(self.target_emb, emb))
            best_sim = max(best_sim, sim)

        if best_sim >= self.threshold:
            text = f"{self.target_name} detectada (similitud={best_sim:.3f})"
            self.get_logger().info(f"✅ {text}")
            out = String()
            out.data = text
            self.pub_result.publish(out)
        else:
            # Si quieres ver las similitudes aunque no pasen el umbral:
            self.get_logger().debug(
                f"Sin coincidencia. Mejor similitud={best_sim:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

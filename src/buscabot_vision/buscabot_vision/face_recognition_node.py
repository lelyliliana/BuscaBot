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

# Carpeta compartida con la galería y la web
GALLERY_DIR = Path.home() / ".buscabot_face_gallery"
GALLERY_DIR.mkdir(parents=True, exist_ok=True)

# Archivo donde vamos a guardar el último estado de reconocimiento
STATUS_PATH = GALLERY_DIR / "face_status.txt"


def load_target_embedding(name: str) -> np.ndarray:
    """
    Cargar embedding desde ~/.buscabot_face_gallery/<name>.json
    """
    json_path = GALLERY_DIR / f"{name}.json"

    if not json_path.exists():
        raise FileNotFoundError(f"No existe la galería para {name}: {json_path}")

    data = json.loads(json_path.read_text())
    emb = np.array(data["embedding"], dtype=np.float32)

    # Normalizar embedding
    norm = np.linalg.norm(emb)
    if norm == 0:
        raise RuntimeError(f"El embedding de {name} tiene norma 0")
    emb = emb / norm

    return emb


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # -------------------------------
        # Parámetro del objetivo actual
        # -------------------------------
        self.target_name = self.declare_parameter(
            'target_name', 'target_current'
        ).get_parameter_value().string_value

        # Umbral base
        self.similarity_threshold = self.declare_parameter(
            'similarity_threshold', 0.35
        ).get_parameter_value().double_value

        # Margen adicional
        self.margin = 0.05

        # -------------------------------
        # Cargar embedding del objetivo
        # -------------------------------
        self.get_logger().info(f"Cargando embedding de: {self.target_name}")
        self.target_embedding = load_target_embedding(self.target_name)

        # -------------------------------
        # Cargar nombre legible desde JSON
        # -------------------------------
        json_path = GALLERY_DIR / f"{self.target_name}.json"
        data = json.loads(json_path.read_text())

        # Soportar formato nuevo ("name") y antiguo ("person_name")
        self.person_label = data.get("name", data.get("person_name", "Persona buscada"))

        self.get_logger().info(
            f"Embedding cargado. Norma = {np.linalg.norm(self.target_embedding):.3f}"
        )

        # -------------------------------
        # Inicializar InsightFace
        # -------------------------------
        self.get_logger().info("Inicializando modelo InsightFace (buffalo_l)...")

        self.app = FaceAnalysis(name="buffalo_l")
        self.app.prepare(ctx_id=0, det_size=(640, 480))

        self.get_logger().info("Modelo InsightFace listo.")

        # -------------------------------
        # ROS2: Suscriptor y Publicador
        # -------------------------------
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )
        self.pub_result = self.create_publisher(String, "/face_recognition", 10)

        # Inicializar archivo de estado
        try:
            STATUS_PATH.write_text("Sin datos aún de la cámara.\n")
        except Exception as e:
            self.get_logger().warn(f"No pude inicializar STATUS_PATH: {e}")

        self.get_logger().info(
            f"FaceRecognitionNode inicializado. "
            f"Objetivo: {self.person_label}, "
            f"umbral: {self.similarity_threshold}"
        )

    # =====================================================
    # CALLBACK PRINCIPAL
    # =====================================================
    def image_callback(self, msg: Image):
        # Convertir a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detectar rostros
        faces = self.app.get(frame)
        if not faces:
            # Si no hay caras, guardamos este estado solo de forma suave (sin spamear)
            try:
                STATUS_PATH.write_text("No se detectan rostros en la imagen actual.\n")
            except Exception as e:
                self.get_logger().warn(f"No pude escribir STATUS_PATH: {e}")
            return

        best_sim = -1.0

        for face in faces:
            emb = face.normed_embedding
            sim = float(np.dot(self.target_embedding, emb))
            best_sim = max(best_sim, sim)

        effective_threshold = self.similarity_threshold + self.margin

        if best_sim >= effective_threshold:
            text = f"{self.person_label} detectada (similitud={best_sim:.3f})"
            self.get_logger().info(f"✔ {text}")

            out = String()
            out.data = text
            self.pub_result.publish(out)

            # 🔥 Guardar también en el archivo de estado
            try:
                STATUS_PATH.write_text(text + "\n")
            except Exception as e:
                self.get_logger().warn(f"No pude escribir STATUS_PATH: {e}")

        else:
            text = f"No coincide con {self.person_label} (similitud={best_sim:.3f})"
            self.get_logger().debug(text)

            out = String()
            out.data = text
            self.pub_result.publish(out)

            # 🔥 Guardar estado de "no coincidencia"
            try:
                STATUS_PATH.write_text(text + "\n")
            except Exception as e:
                self.get_logger().warn(f"No pude escribir STATUS_PATH: {e}")


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

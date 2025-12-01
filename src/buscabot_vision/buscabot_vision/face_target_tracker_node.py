import json
import os
from typing import Optional, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from insightface.app import FaceAnalysis


GALLERY_DIR = os.path.expanduser("~/.buscabot_face_gallery")


class FaceTargetTrackerNode(Node):
    def __init__(self):
        super().__init__('face_target_tracker_node')

        # --- Parámetros -------------------------------------------------
        # nombre de la persona a buscar (coincide con el .json de la galería)
        self.declare_parameter('target_name', 'Leli')
        self.declare_parameter('similarity_threshold', 0.55)

        self.target_name: str = (
            self.get_parameter('target_name').get_parameter_value().string_value
        )
        self.similarity_threshold: float = (
            self.get_parameter('similarity_threshold').get_parameter_value().double_value
        )

        # --- Bridge OpenCV <-> ROS --------------------------------------
        self.bridge = CvBridge()

        # --- Cargamos InsightFace ---------------------------------------
        self.get_logger().info("🧠 Cargando modelo InsightFace (buffalo_l, CPU)...")
        self.app = FaceAnalysis(
            name='buffalo_l',
            root=os.path.expanduser("~/.insightface"),
            providers=['CPUExecutionProvider'],
        )
        # det-size pequeño para ir rápido con la cámara
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.get_logger().info("✅ Modelo InsightFace inicializado")

        # --- Cargamos el embedding objetivo -----------------------------
        self.target_embedding: Optional[np.ndarray] = self._load_target_embedding(
            self.target_name
        )

        # --- Suscripción a /image_raw -----------------------------------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.process_frame,
            qos,
        )

        # --- Log de configuración ---------------------------------------
        self.get_logger().info(
            f"🎯 Objetivo cargado: {self.target_name} "
            f"(dim = {self.target_embedding.shape[0] if self.target_embedding is not None else 'None'})"
        )
        self.get_logger().info(
            f"🔧 Umbral de similitud configurado en {self.similarity_threshold:.3f}"
        )
        self.get_logger().info("🚀 Tracker listo. Esperando imágenes de /image_raw ...")

    # ------------------------------------------------------------------ #
    #  Cargar embedding desde ~/.buscabot_face_gallery
    # ------------------------------------------------------------------ #
    def _load_target_embedding(self, name: str) -> np.ndarray:
        if not os.path.isdir(GALLERY_DIR):
            raise RuntimeError(
                f"No se encontró el directorio de galería {GALLERY_DIR}. "
                f"Asegúrate de haber enrolado previamente a {name}."
            )

        filename = os.path.join(GALLERY_DIR, f"{name}.json")
        if not os.path.isfile(filename):
            raise RuntimeError(
                f"No se encontró el archivo de embedding {filename}. "
                f"Asegúrate de haber enrolado a {name}."
            )

        with open(filename, 'r') as f:
            data = json.load(f)

        if 'embedding' not in data:
            raise RuntimeError(f"El archivo {filename} no contiene 'embedding'.")

        emb = np.array(data['embedding'], dtype=np.float32)
        # normalizamos
        norm = np.linalg.norm(emb)
        if norm > 0:
            emb = emb / norm

        return emb

    # ------------------------------------------------------------------ #
    #  Similitud coseno
    # ------------------------------------------------------------------ #
    def cosine_similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        a = a.astype(np.float32)
        b = b.astype(np.float32)

        a_norm = np.linalg.norm(a)
        b_norm = np.linalg.norm(b)

        if a_norm == 0.0 or b_norm == 0.0:
            return 0.0

        a = a / a_norm
        b = b / b_norm

        return float(np.dot(a, b))

    # ------------------------------------------------------------------ #
    #  Callback de la cámara
    # ------------------------------------------------------------------ #
    def process_frame(self, msg: Image):
        # Convertimos ROS -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detectamos caras con InsightFace
        faces: List = self.app.get(frame)

        if not faces:
            self.get_logger().info("📷 Frame sin caras | found=False", throttle_duration_sec=2.0)
            return

        # Tomamos la cara con mayor score
        faces_sorted = sorted(faces, key=lambda f: f['det_score'], reverse=True)
        best_face = faces_sorted[0]

        # Embedding de esa cara
        emb = best_face['embedding'].astype(np.float32)
        # normalizamos igual que el target
        norm = np.linalg.norm(emb)
        if norm > 0:
            emb = emb / norm

        sim = self.cosine_similarity(emb, self.target_embedding)

        if sim >= self.similarity_threshold:
            self.get_logger().info(
                f"🎯 Cara objetivo ENCONTRADA | similitud={sim:.3f} >= {self.similarity_threshold:.3f} | found=True"
            )
        else:
            self.get_logger().info(
                f"🙈 Cara distinta | similitud={sim:.3f} < {self.similarity_threshold:.3f} | found=False",
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = FaceTargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

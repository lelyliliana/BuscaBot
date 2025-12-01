#!/usr/bin/env python3
import numpy as np
import cv2
import insightface

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaceEnrollerNode(Node):
    def __init__(self):
        super().__init__("face_enroller_node")

        # Parámetro: nombre de la persona
        self.declare_parameter("person_name", "persona_desconocida")
        self.person_name = (
            self.get_parameter("person_name")
            .get_parameter_value()
            .string_value
        )

        # Cuántas caras queremos acumular
        self.n_faces_target = 30
        self.embeddings = []

        # Bridge ROS ↔ OpenCV
        self.bridge = CvBridge()

        # Cargar modelo InsightFace (buffalo_l) SOLO CPU
        self.get_logger().info(
            f"🧠 Cargando modelo InsightFace (buffalo_l) para {self.person_name}..."
        )
        self.app = insightface.app.FaceAnalysis(
            name="buffalo_l",
            providers=["CPUExecutionProvider"],
        )
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.get_logger().info("✅ Modelo InsightFace cargado.")

        # 🔴 IMPORTANTE: QoS normal (RELIABLE), nada de sensor_data_qos
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info(
            f"📡 Suscrito a /image_raw, esperando imágenes para enrolar a: {self.person_name}"
        )

    def image_callback(self, msg: Image):
        """Se ejecuta cada vez que llega una imagen por /image_raw."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        faces = self.app.get(frame)
        if not faces:
            return

        face = max(
            faces,
            key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]),
        )

        emb = face.embedding  # vector de 512
        self.embeddings.append(emb)

        self.get_logger().info(
            f"📸 Cara detectada. Muestras acumuladas: "
            f"{len(self.embeddings)}/{self.n_faces_target}"
        )

        if len(self.embeddings) >= self.n_faces_target:
            self.finish_enrollment()

    def finish_enrollment(self):
        """Promedia embeddings y guarda el resultado."""
        from pathlib import Path

        self.get_logger().info("🧮 Calculando embedding promedio...")

        embs = np.stack(self.embeddings, axis=0)
        avg_emb = np.mean(embs, axis=0)

        base_dir = Path.home() / ".insightface" / "buscabot_faces"
        base_dir.mkdir(parents=True, exist_ok=True)
        file_path = base_dir / f"{self.person_name}.npz"

        np.savez(str(file_path), embedding=avg_emb, person=self.person_name)

        self.get_logger().info(
            f"✅ Enrolamiento completado para '{self.person_name}'. "
            f"Archivo guardado en: {file_path}"
        )

        self.get_logger().info("🛑 Cerrando nodo face_enroller_node...")
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
            rclpy.shutdown()


if __name__ == "__main__":
    main()

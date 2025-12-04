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
TARGET_JSON = GALLERY_DIR / "target_current.json"
last_target_mtime = None   # Para saber si el archivo cambió
current_target_name = None
current_target_emb = None

def cargar_objetivo_actual():
    """
    Lee ~/.buscabot_face_gallery/target_current.json
    y actualiza las variables globales si el archivo cambió.
    Devuelve:
        True  -> si se recargó un objetivo nuevo
        False -> si no hubo cambios
    """
    global last_target_mtime, current_target_name, current_target_emb

    if not TARGET_JSON.exists():
        # Si aún no hay objetivo guardado, no hacemos nada
        return False

    # Comprobar si el archivo cambió (mtime = fecha de modificación)
    mtime = TARGET_JSON.stat().st_mtime
    if last_target_mtime is not None and mtime == last_target_mtime:
        # No ha cambiado
        return False

    # Leer JSON
    data = json.loads(TARGET_JSON.read_text())

    # Nombre legible de la persona
    nombre = data.get("person_name", "Persona buscada")

    # Embedding como vector numpy y normalizado
    emb = np.array(data["embedding"], dtype=np.float32)
    norm = np.linalg.norm(emb)
    if norm == 0:
        raise RuntimeError("El embedding cargado tiene norma 0")

    emb = emb / norm

    # Actualizar variables globales
    current_target_name = nombre
    current_target_emb = emb
    last_target_mtime = mtime

    return True


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
        # Parámetro: umbral base
        self.similarity_threshold = self.declare_parameter(
            'similarity_threshold', 0.35
        ).get_parameter_value().double_value

        # Margen adicional para evitar falsos positivos
        self.margin = 0.05

        # Cargar objetivo inicial desde target_current.json
        self.get_logger().info("Cargando objetivo inicial desde target_current.json ...")
        changed = cargar_objetivo_actual()

        # Si no cambió pero tampoco hay nada cargado, es un error
        if current_target_emb is None:
            raise RuntimeError(
                "No se pudo cargar ningún objetivo desde "
                f"{TARGET_JSON}. Sube primero una foto con la web."
            )

        # Guardar en atributos de la instancia
        self.person_label = current_target_name or "Persona buscada"
        self.target_embedding = current_target_emb

        self.get_logger().info(
            f"Objetivo inicial: {self.person_label} "
            f"(norma = {np.linalg.norm(self.target_embedding):.3f})"
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

        # 👀 Revisar si cambió el archivo target_current.json
        changed = cargar_objetivo_actual()
        if changed:
            # Actualizar objetivo en este nodo
            self.person_label = current_target_name or "Persona buscada"
            self.target_embedding = current_target_emb

            self.get_logger().info(
                f"Nuevo objetivo cargado: {self.person_label} "
                f"(norma = {np.linalg.norm(self.target_embedding):.3f})"
            )

            # Opcional: escribir un mensaje especial en el archivo de estado
            try:
                STATUS_PATH.write_text(f"Nuevo objetivo cargado: {self.person_label}\n")
            except Exception as e:
                self.get_logger().warn(f"No pude escribir STATUS_PATH: {e}")

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

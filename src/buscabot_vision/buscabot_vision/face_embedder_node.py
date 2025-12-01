import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from insightface.app import FaceAnalysis


class FaceEmbedderNode(Node):
    def __init__(self):
        super().__init__('face_embedder_node')

        self.bridge = CvBridge()

        self.get_logger().info("Cargando modelo InsightFace (buffalo_l) en CPU...")
        self.app = FaceAnalysis(name='buffalo_l', providers=['CPUExecutionProvider'])
        self.app.prepare(ctx_id=-1, det_size=(640, 640))
        self.get_logger().info("Modelo InsightFace cargado.")

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.frame_count = 0

    def image_callback(self, msg: Image):
        # Convierte ROS -> OpenCV, pero SIN mostrar ventana
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Para no saturar: procesa 1 de cada 5 frames
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return

        faces = self.app.get(frame)

        if len(faces) == 0:
            self.get_logger().info("No se detectaron rostros en este frame.")
            return

        self.get_logger().info(f"Se detectaron {len(faces)} rostro(s).")

        for i, face in enumerate(faces):
            emb = face.embedding
            self.get_logger().info(
                f"Rostro {i}: embedding (primeros 5 valores): {emb[:5]}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FaceEmbedderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

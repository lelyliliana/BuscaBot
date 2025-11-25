import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import numpy as np


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.bridge = CvBridge()
        self.saved_frame = False      # para guardar solo un frame de depuración
        self.logged_encoding = False  # para ver qué encoding llega

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publicador del resultado
        self.faces_pub = self.create_publisher(String, '/faces', 10)

        # Parámetro para ventana (por ahora no la usamos)
        self.declare_parameter('show_window', False)
        self.show_window = self.get_parameter(
            'show_window'
        ).get_parameter_value().bool_value

        # Buscar el Haarcascade
        possible_paths = [
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/local/lib/python3.10/site-packages/cv2/data/haarcascade_frontalface_default.xml',
            '/usr/lib/python3/dist-packages/cv2/data/haarcascade_frontalface_default.xml',
        ]

        haar_path = None
        for p in possible_paths:
            if os.path.exists(p):
                haar_path = p
                break

        if haar_path is None:
            self.get_logger().error(
                'No se encontró haarcascade_frontalface_default.xml'
            )
            self.face_cascade = None
        else:
            self.get_logger().info(f'Usando cascade en: {haar_path}')
            self.face_cascade = cv2.CascadeClassifier(haar_path)

        self.get_logger().info('Nodo face_detector inicializado.')

    def image_callback(self, msg: Image):
        # Ver encoding que llega desde v4l2_camera SOLO una vez
        if not self.logged_encoding:
            self.get_logger().info(f'Encoding recibido: {msg.encoding}')
            self.logged_encoding = True

        if self.face_cascade is None or self.face_cascade.empty():
            self.get_logger().error(
                'El clasificador de rostros no está cargado correctamente.'
            )
            return

        # --- Conversión del mensaje ROS a imagen OpenCV ---

        # Si viene en YUYV (yuv422_yuy2), lo convertimos a BGR "a mano"
        if msg.encoding.startswith('yuv422') or msg.encoding.startswith('yuyv'):
            # datos brutos → arreglo numpy
            yuv = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                yuv = yuv.reshape((msg.height, msg.width, 2))
            except ValueError:
                self.get_logger().error(
                    f'No se pudo hacer reshape de la imagen: '
                    f'h={msg.height}, w={msg.width}, len(data)={len(msg.data)}'
                )
                return

            # Conversión correcta de YUYV → BGR
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
        else:
            # Para otros encodings, usamos cv_bridge normalmente
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Guardar un frame de depuración solo una vez
        if not self.saved_frame:
            debug_path = '/home/buscabot/frame_debug2.jpg'
            cv2.imwrite(debug_path, frame)
            self.get_logger().info(f'Imagen de depuración guardada en: {debug_path}')
            self.saved_frame = True

        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar rostros
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.05,
            minNeighbors=3,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        # Publicar resultado
        faces_list = [(int(x), int(y), int(w_), int(h_)) for (x, y, w_, h_) in faces]
        out_msg = String()
        out_msg.data = f'N_FACES={len(faces_list)}; {faces_list}'
        self.faces_pub.publish(out_msg)

        self.get_logger().info(f'Rostros detectados: {len(faces_list)}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.bridge = CvBridge()

        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',   # Si tu cámara usa otro tópico, luego lo cambiamos
            self.image_callback,
            10
        )

        # Publicador de resultados
        self.faces_pub = self.create_publisher(String, '/faces', 10)

        # Parámetro para mostrar o no la ventana
        self.declare_parameter('show_window', False)
        self.show_window = self.get_parameter(
            'show_window'
        ).get_parameter_value().bool_value

        # Cargar el clasificador Haar de rostros
        haar_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(haar_path):
            self.get_logger().error(f'No se encontró el archivo Haar Cascade en: {haar_path}')
        self.face_cascade = cv2.CascadeClassifier(haar_path)

        self.get_logger().info('Nodo de detección de rostros inicializado.')

    def image_callback(self, msg: Image):
        # ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detección de rostros
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(40, 40)
        )

        # Dibujar rectángulos para depuración
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Enviar resultado por tópico
        faces_list = [(int(x), int(y), int(w), int(h)) for (x, y, w, h) in faces]
        out = String()
        out.data = f'N_FACES={len(faces_list)}; {faces_list}'
        self.faces_pub.publish(out)

        if self.show_window:
            cv2.imshow('BuscaBot Faces', frame)
            cv2.waitKey(1)

        # Log opcional
        # self.get_logger().info(f'Se detectaron {len(faces_list)} rostros.')


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


if __name__ == '__main__':
    main()

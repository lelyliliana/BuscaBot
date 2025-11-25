import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArgusCameraNode(Node):
    def __init__(self):
        super().__init__('argus_camera_node')

        # Parámetros básicos (podemos afinarlos luego)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)

        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        framerate = self.get_parameter('framerate').get_parameter_value().integer_value

        self.get_logger().info(
            f'Iniciando ArgusCameraNode con {width}x{height} @ {framerate} FPS'
        )

        # Pipeline GStreamer con nvarguscamerasrc
        self.pipeline = (
            f"nvarguscamerasrc ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={framerate}/1 ! "
            f"nvvidconv flip-method=0 ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=true max-buffers=1"
        )

        self.get_logger().info(f'Usando pipeline:\n{self.pipeline}')

        # Abrir cámara con OpenCV + GStreamer
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara con nvarguscamerasrc.')
            raise RuntimeError('No se pudo abrir la cámara')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)

        # Timer para capturar frames
        self.timer_period = 1.0 / float(framerate)
        self.timer = self.create_timer(self.timer_period, self.capture_frame)

        self.get_logger().info('Nodo ArgusCameraNode inicializado correctamente.')

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('No se pudo leer frame de la cámara.')
            return

        # Convertir frame (BGR) a mensaje ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.image_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Cerrando cámara Argus...')
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArgusCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')

        # Publicamos en el mismo tópico que usa tu nodo de caras
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Inicializar GStreamer
        Gst.init(None)

        # MISMA tubería que te funciona con gst-launch, pero terminando en appsink
        self.pipeline_str = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=360, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink emit-signals=True max-buffers=1 drop=True"
        )

        self.get_logger().info("Lanzando pipeline GStreamer para la IMX219...")
        self.pipeline = Gst.parse_launch(self.pipeline_str)

        # Obtenemos el appsink
        self.sink = self.pipeline.get_by_name('sink')
        if self.sink is None:
            self.get_logger().error("No se pudo obtener el elemento 'sink' de la tubería.")
            raise RuntimeError("appsink no encontrado")

        # Conectamos la señal new-sample
        self.sink.connect('new-sample', self.on_new_sample)

        # Arrancamos la tubería
        state_ret = self.pipeline.set_state(Gst.State.PLAYING)
        if state_ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("No se pudo poner la tubería en estado PLAYING.")
            raise RuntimeError("Fallo al iniciar GStreamer")

        self.get_logger().info("✅ Pipeline GStreamer en PLAYING, esperando frames de la IMX219...")

    def on_new_sample(self, sink):
        """Callback que se ejecuta cada vez que llega un frame al appsink."""
        sample = sink.emit('pull-sample')
        if sample is None:
            self.get_logger().warn("⚠️ sample == None en appsink")
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        if caps is None:
            self.get_logger().warn("⚠️ caps == None en sample")
            return Gst.FlowReturn.OK

        structure = caps.get_structure(0)
        width = structure.get_value('width')
        height = structure.get_value('height')

        # Pasamos de buffer GStreamer → bytes → numpy
        data = buf.extract_dup(0, buf.get_size())
        frame = np.ndarray(shape=(height, width, 3), buffer=data, dtype=np.uint8)

        # Publicamos en ROS
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.publisher_.publish(msg)

        return Gst.FlowReturn.OK

    def destroy_node(self):
        # Apagamos la tubería limpito
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


class CsiCameraNode(Node):
    """
    Nodo de cámara para Jetson con IMX219 (CSI).
    Usa nvarguscamerasrc + GStreamer y publica en /image_raw (BGR8).
    """

    def __init__(self):
        super().__init__("csi_camera_node")

        # Publisher al mismo tópico que usan tus nodos de visión
        self.publisher_ = self.create_publisher(Image, "/image_raw", 10)
        self.bridge = CvBridge()

        # Inicializar GStreamer
        Gst.init(None)

        # Pipeline muy similar al que ya probaste con gst-launch
        self.pipeline_str = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1920, height=1080, "
            "format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=360, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink name=sink emit-signals=True max-buffers=1 drop=True"
        )

        self.get_logger().info(
            f"Iniciando cámara CSI con GStreamer:\n  {self.pipeline_str}"
        )

        # Crear pipeline
        self.pipeline = Gst.parse_launch(self.pipeline_str)

        # Obtener el appsink
        self.sink = self.pipeline.get_by_name("sink")
        if self.sink is None:
            self.get_logger().error("❌ No se pudo obtener el elemento 'sink'")
            raise RuntimeError("appsink no encontrado en pipeline")

        # Conectar callback para nuevos frames
        self.sink.connect("new-sample", self.on_new_sample)

        # Arrancar pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret != Gst.StateChangeReturn.SUCCESS and ret != Gst.StateChangeReturn.ASYNC:
            self.get_logger().error(
                f"❌ No se pudo poner la pipeline en PLAYING (ret={ret})"
            )
            raise RuntimeError("No se pudo iniciar pipeline GStreamer")

        self.get_logger().info("✅ Cámara CSI iniciada correctamente via GStreamer")

        # Bus de mensajes (opcionalmente podríamos leer errores por aquí)
        self.bus = self.pipeline.get_bus()

    # ------------------------------------------------------------------ #
    # CALLBACK DE GSTREAMER: se ejecuta cuando llega un nuevo frame
    # ------------------------------------------------------------------ #
    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            self.get_logger().warn("⚠️ Sample nulo desde appsink")
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")

        # Extraer datos crudos del buffer
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().warn("⚠️ No se pudo mapear el buffer de video")
            return Gst.FlowReturn.OK

        try:
            # Construir frame BGR de NumPy (height x width x 3)
            frame = np.ndarray(
                (height, width, 3),
                buffer=map_info.data,
                dtype=np.uint8,
            )
            # Copia defensiva para no depender del buffer de GStreamer
            frame_bgr = frame.copy()
        finally:
            buf.unmap(map_info)

        # Convertir a mensaje ROS y publicar
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ Error publicando frame: {e}")

        return Gst.FlowReturn.OK

    def destroy_node(self):
        # Apagar pipeline de forma limpia
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

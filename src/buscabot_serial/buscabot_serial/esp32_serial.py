import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial

class ESP32SerialNode(Node):
    def __init__(self):
        super().__init__('esp32_serial')

        # Serial con ESP32
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Suscriptor a /cmd_vel (velocidades deseadas desde Jetson)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publicador de estado de motores
        self.motor_pub = self.create_publisher(String, '/motor_status', 10)

        # Timer para leer del ESP32 cada 0.1s
        self.timer = self.create_timer(0.1, self.read_serial)

    def cmd_vel_callback(self, msg):
        # Convertir Twist en una cadena simple para enviar al ESP32
        command = f"{msg.linear.x},{msg.angular.z}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Enviado a ESP32: {command.strip()}")

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.motor_pub.publish(msg)
                    self.get_logger().info(f"ESP32 -> {line}")
            except Exception as e:
                self.get_logger().warn(f"Error leyendo serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

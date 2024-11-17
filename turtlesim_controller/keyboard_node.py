# turtle_controller/keyboard_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import time
import select


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher = self.create_publisher(String, 'keyboard', 10)
        self.get_logger().info("Nodo de teclado iniciado. Publicando teclas actualmente presionadas.")

        # Configuración de terminal para lectura de teclas
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Set para mantener teclas presionadas
        self.pressed_keys = set()

    def update_pressed_keys(self):
        # Lee teclas presionadas y liberadas
        while select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1).lower()
            if key == '\x03':  # Ctrl-C para salir
                return False  # Indicador de que se debe salir

            # Añadir o quitar tecla del conjunto según estado
            if key in self.pressed_keys:
                self.pressed_keys.remove(key)
            else:
                self.pressed_keys.add(key)

        return True

    def publish_pressed_keys(self):
        # Publica todas las teclas actualmente presionadas como un string
        if not self.update_pressed_keys():
            return False  # Indicador de que se debe salir

        msg = String()
        msg.data = ''.join(sorted(self.pressed_keys))
        self.publisher.publish(msg)
        return True

    def run(self):
        try:
            while rclpy.ok():
                # Publicar solo si hay cambios en las teclas presionadas
                if not self.publish_pressed_keys():
                    break
                time.sleep(0.05)  # Pequeño retraso para no sobrecargar la CPU
        finally:
            # Restaurar configuración de terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    try:
        keyboard_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()

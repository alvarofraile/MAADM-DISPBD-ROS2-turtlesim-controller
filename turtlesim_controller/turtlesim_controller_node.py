#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty  # Para el servicio de limpieza de trazos
from turtlesim.srv import SetPen  # Servicio para cambiar el color de la traza
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.srv import SetPen, TeleportAbsolute  # Servicio para mover la tortuga
import math
from rcl_interfaces.msg import SetParametersResult

class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        self.declare_parameter('speed', 1.0)

        self.speed = self.get_parameter('speed').value
        self.rotation_speed = 1.0
        self.drawing_enabled = True
        self.clear_drawn_lines = False
        self.position_reset_status = False

        self.char_subscriber = self.create_subscription(String, "keyboard", self.process_action, 10)
        self.ts_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Cliente del servicio de limpieza
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio /clear esté disponible...')

        # Cliente del servicio para cambiar el color de la traza
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio /turtle1/set_pen esté disponible...')

        # Cliente del servicio para teletransportar la tortuga
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio /turtle1/teleport_absolute esté disponible...')

        # Callback para detectar cambios en los parámetros
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().info("Controlador Turtlesim creado con éxito")

    def on_parameter_change(self, params):
        """Callback que se activa cuando cambia algún parámetro."""
        for param in params:
            if param.name == 'speed' and param.type_ == param.Type.DOUBLE:
                try:
                    self.speed = float(param.value)
                    self.get_logger().info(f"Parámetro 'speed' cambiado a {self.speed}")
                except ValueError:
                    self.get_logger().error(f"Valor inválido para 'speed': {param.value}")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def process_action(self, msg):
        characters = str(msg.data)
        twist = Twist()
        movement = 0.0
        rotation = 0.0

        if "w" in characters:
            movement += self.speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Forward")
        if "s" in characters:
            movement += -self.speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Backward")
        if "a" in characters:
            rotation += self.rotation_speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Rotate Left")
        if "d" in characters:
            rotation += -self.rotation_speed
            self.get_logger().info("Rotate Right")

        if " " not in characters:
            self.toggle_drawing(True)
        else:
            self.toggle_drawing(False)

        if "c" in characters:
            self.clear_drawn(True)
        else:
            self.clear_drawn(False)

        if "r" in characters:
            self.reset_position(True)
        else:
            self.reset_position(False)
            twist.linear.x = movement
            twist.angular.z = rotation

            self.ts_publisher.publish(twist)

    def toggle_drawing(self, status):
        # Alterna el estado de dibujo
        self.drawing_enabled = status

        request = SetPen.Request()

        if not self.drawing_enabled:
            # Cambia el color de la traza al color de fondo (simula que está desactivada)
            request.r = 69
            request.g = 86
            request.b = 255
        else:
            # Restaura el color de la traza (blanco por defecto)
            request.r = 255
            request.g = 255
            request.b = 255

        # Parámetros adicionales para mantener el ancho de la traza y activar la traza
        request.width = 2
        request.off = False
        self.set_pen_client.call_async(request)

    def clear_drawn(self, status):
        self.clear_drawn_lines = status

        if(self.clear_drawn_lines):
            # Llama al servicio de limpieza
            request = Empty.Request()
            self.clear_client.call_async(request)
            self.get_logger().info("Limpieza de trazos")

    def reset_position(self, status):
        self.position_reset_status = status

        if(self.position_reset_status):
            # Teletransporta la tortuga al centro (5.5, 5.5) y orienta hacia 90 grados
            request = TeleportAbsolute.Request()
            request.x = 5.5
            request.y = 5.5
            request.theta = math.radians(90)  # Orientación hacia arriba en radianes

            self.teleport_client.call_async(request)

def main(args=None):
    try:
        rclpy.init(args=args)
        turtlesim_controller = TurtlesimController()
        rclpy.spin(turtlesim_controller)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
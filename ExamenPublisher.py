import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from pyzbar.pyzbar import decode


class PioneerSimpleBehavior(Node):
    def __init__(self):
        super().__init__('pioneer_smart_behavior')
        self.get_logger().info('Pioneer simple behavior started!')

        # Publicador de velocidad
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Diccionario de acciones según la letra del QR
        self.actions = {
            'A': 'Der90',
            'B': 'Izq90',
            'G': 'Girar180Detener',
            'H': 'Vuelta180',
            'C': 'Izq90'
        }

        # Diccionario de mensajes de ID para cada acción
        self.messages = {
            'A': 'ID 1 - giro 90 derecha',
            'B': 'ID 0 - giro 90 izquierda',
            'H': 'ID 2 - giro 180 grados',
            'C': 'ID 3 - giro 90 izquierda',
            'G': 'ID 4 - giro 180 y FIN'
        }

        # Inicializar cámara para lectura de QR
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara. Verifica la conexión o índice.")
            raise RuntimeError("Cámara no disponible. Nodo cerrado.")

        # Ajustar resolución de la cámara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Variables de control
        self.is_executing_action = False
        self.allow_reversing = True
        self.last_qr_action = None

        # Temporizador principal
        self.timer = self.create_timer(0.1, self.run_behavior)

    def stop_movement(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(twist_msg)

    def reverse_movement(self):
        if self.allow_reversing and not self.is_executing_action:
            twist_msg = Twist()
            twist_msg.linear.x = -0.05
            twist_msg.angular.z = 0.0
            self.pub_cmd_vel.publish(twist_msg)

    def execute_action(self, action):
        if self.is_executing_action:
            return

        self.is_executing_action = True
        self.allow_reversing = False
        twist_msg = Twist()

        if action == 'Der90':
            twist_msg.angular.z = -1.8
            self.get_logger().info('Girar a la derecha 90°')
        elif action == 'Izq90':
            twist_msg.angular.z = 1.8
            self.get_logger().info('Girar a la izquierda 90°')
        elif action == 'Vuelta180':
            twist_msg.angular.z = 3.6
            self.get_logger().info('Girar 180°')
        elif action == 'Girar180Detener':
            twist_msg.angular.z = 3.6
            self.get_logger().info('ID 4 - giro 180 y FIN')
            self.pub_cmd_vel.publish(twist_msg)
            self.create_timer(2.0, self.finish_action_with_stop)
            return
        elif action == 'Alto':
            self.allow_reversing = False
            self.get_logger().info('Alto')
            self.stop_movement()
            self.is_executing_action = False
            return

        self.pub_cmd_vel.publish(twist_msg)
        action_duration = 2.0 if action == 'Vuelta180' else 1.0
        self.create_timer(action_duration, self.finish_action)

    def finish_action(self):
        self.stop_movement()
        self.is_executing_action = False
        if self.last_qr_action != 'Alto':
            self.allow_reversing = True

    def finish_action_with_stop(self):
        self.stop_movement()
        self.is_executing_action = False
        self.allow_reversing = False

    def read_qr(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer el frame de la cámara.")
            return None

        frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=30)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

        for codes in decode(gray_frame):
            info = codes.data.decode('utf-8')
            letra = info[0]
            action = self.actions.get(letra, None)

            if len(codes.polygon) >= 4:
                area = cv2.contourArea(np.array(codes.polygon))
                if area < 164000:
                    self.get_logger().info(f'Área del QR: {area}')
                    return None

                if action != self.last_qr_action:
                    self.last_qr_action = action
                    self.get_logger().info(self.messages.get(letra, 'Acción no definida'))
                    return action

        return None

    def run_behavior(self):
        if self.is_executing_action:
            return

        action = self.read_qr()
        if action is None:
            self.reverse_movement()
        else:
            self.stop_movement()
            self.execute_action(action)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PioneerSimpleBehavior()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


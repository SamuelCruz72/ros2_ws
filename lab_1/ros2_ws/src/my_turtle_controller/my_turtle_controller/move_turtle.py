import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty 

import threading
import sys
import tty
import termios
import time

class TurtleController(Node):
    def __init__(self):
        # Inicializa el nodo de la tortuga
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move_turtle)
        self.last_key_time = time.time()

        # Inicializa clientes de servicio
        self.current_twist = Twist()
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_absolute...')
        
        # Inicia el hilo para escuchar el teclado
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard, daemon = True)
        self.keyboard_thread.start()
    
    # Recibe indicaciones de movimiento de la tortuga
    def move_turtle(self):
        if time.time() - self.last_key_time > 2:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    # Fija las condiciones del lápiz
    def set_pen(self, r=255, g=255, b=255, width=3, off=False):
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servicio de lápiz no disponible.')
            return
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        future = self.pen_client.call_async(request)
        future.add_done_callback(self.pen_response_callback)

    # Indica el estado de la llamada al lápiz
    def pen_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Lápiz actualizado correctamente.")
        except Exception as e:
            self.get_logger().error(f'Error al actualizar el lápiz: {e}')

    # Transporta la tortuga a una posición y pose determinada
    def teleport_to(self, x, y, theta):
        self.last_position_x = x
        self.last_position_y = y
        self.set_pen(off=True)
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = self.teleport_client.call_async(request)
        future.add_done_callback(self.after_teleport)

    # Indica el estado del transporte de la tortuga
    def after_teleport(self, future):
        try:
            future.result()
            self.get_logger().info("Teletransporte completado.")
            time.sleep(0.1)
            self.set_pen(off=False)
        except Exception as e:
            self.get_logger().error(f'Error en el teletransporte: {e}')

    # Borra todos los trazos de la tortuga
    def clear_background(self):
        clear_client = self.create_client(Empty, '/clear')
        if not clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servicio /clear no disponible.')
            return
        request = Empty.Request()
        future = clear_client.call_async(request)

    # Lee el estado del teclado
    def listen_keyboard(self):
        self.get_logger().info("Usa las flechas para mover la tortuga. Presiona 'q' para salir.")
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                self.last_key_time = time.time()
                if key == '\x1b':  # Comienza secuencia de flecha 
                    key2 = sys.stdin.read(1)
                    key3 = sys.stdin.read(1)
                    if key2 == '[':
                        if key3 == 'A':  # Flecha arriba
                            self.get_logger().info("Tortuga avanzando")
                            self.current_twist.linear.x = 2.0
                            self.current_twist.angular.z = 0.0
                        elif key3 == 'B':  # Flecha abajo
                            self.get_logger().info("Tortuga retrocediendo")
                            self.current_twist.linear.x = -2.0
                            self.current_twist.angular.z = 0.0
                        elif key3 == 'C':  # Flecha derecha
                            self.get_logger().info("Tortuga girando a la derecha")
                            self.current_twist.linear.x = 0.0
                            self.current_twist.angular.z = -2.0
                        elif key3 == 'D':  # Flecha izquierda
                            self.get_logger().info("Tortuga girando a la izquierda")
                            self.current_twist.linear.x = 0.0
                            self.current_twist.angular.z = 2.0
                elif key == 's':  
                    self.get_logger().info("Dibujando la letra 'S'")
                    self.teleport_to(4.0, 7.0, 3*math.pi/4) # Transporta a la tortuga al extremo superior derecho de la S
                    time.sleep(0.3) 
                    self.draw_s()  
                elif key == 'a':  
                    self.get_logger().info("Dibujando la letra 'A'")
                    self.teleport_to(4.5, 5.13, math.pi/3) # Transporta a la tortuga al extremo inferior izquierdo de la A
                    time.sleep(0.3) 
                    self.draw_a()  
                elif key == 'c':  
                    self.get_logger().info("Dibujando la letra 'C'")
                    self.teleport_to(8.5, 7.0, 5*math.pi/6) # Transporta a la tortuga al extremo superior derecho de la C
                    time.sleep(0.3)
                    self.draw_c()
                elif key == 'r':
                    self.get_logger().info("Reiniciando: limpiando pantalla y reubicando tortuga")
                    self.clear_background()
                    self.current_twist.linear.x = 0.0
                    self.current_twist.angular.z = 0.0
                    self.teleport_to(5.5, 5.5, 0.0) # Devuelve la tortuga al origen
                elif key == 'q':
                    self.get_logger().info("Saliendo")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def draw_s(self):
        # Dibuja el semicírculo superior de la S
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 14.0
        start_time = time.time()
        while time.time() - start_time < 5*math.pi/(14*4)-0.05:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Detiene la tortuga una vez completa la mitad superior
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        time.sleep(0.2) 
        # Corrige la orientación de la tortuga 
        self.teleport_to(3.65, 6.15, 0.0)
        time.sleep(0.2) 
        # Dibuja el semicírculo inferior de la S
        self.current_twist.angular.z = -14.0
        self.current_twist.linear.x = 7.0
        mid_time = time.time()
        while time.time() - mid_time < 5*math.pi/(14*4):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Detiene la tortuga tras finalizar la letra
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_a(self):
        # Dibuja la línea ascendente de la A
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        rise_time = time.time()
        while time.time() - rise_time < 2.16/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Gira la tortuga para dibujar el flanco descendente
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -7.0
        mid_time = time.time()
        while time.time() - mid_time < 2*math.pi/(3*7):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Dibuja la línea descendente de la A
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        fall_time = time.time()
        while time.time() - fall_time < 2.11/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Detiene la tortuga una vez completa la V invertida
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        time.sleep(0.2) 
        # Transporta la tortuga para dibujar la línea horizontal de la A
        self.teleport_to(5.0, 6.0, 0.0)
        time.sleep(0.2) 
        # Dibuja la línea horizontal de la A
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        last_time = time.time()
        while time.time() - last_time < 0.97/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Detiene la tortuga tras finalizar la letra  
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_c(self):
        # Dibuja el segmento de círculo de la C
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 7.0
        start_time = time.time()
        while time.time() - start_time < 4*math.pi/(7*3):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        # Detiene la tortuga tras finalizar la letra  
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

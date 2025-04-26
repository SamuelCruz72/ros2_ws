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
    def _init_(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move_turtle)
        self.last_key_time = time.time()

        self.current_twist = Twist()
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_absolute...')
        
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard, daemon = True)
        self.keyboard_thread.start()

    def move_turtle(self):
        if time.time() - self.last_key_time > 2:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

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

    def pen_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Lápiz actualizado correctamente.")
        except Exception as e:
            self.get_logger().error(f'Error al actualizar el lápiz: {e}')

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

    def after_teleport(self, future):
        try:
            future.result()
            self.get_logger().info("Teletransporte completado.")
            time.sleep(0.1)
            self.set_pen(off=False)
        except Exception as e:
            self.get_logger().error(f'Error en el teletransporte: {e}')

    def clear_background(self):
        clear_client = self.create_client(Empty, '/clear')
        if not clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servicio /clear no disponible.')
            return
        request = Empty.Request()
        clear_client.call_async(request)

    def listen_keyboard(self):
        self.get_logger().info("Usa las flechas para mover la tortuga. Presiona 'q' para salir.")
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                self.last_key_time = time.time()
                if key == '\x1b':
                    key2 = sys.stdin.read(1)
                    key3 = sys.stdin.read(1)
                    if key2 == '[':
                        if key3 == 'A':
                            self.get_logger().info("Tortuga avanzando")
                            self.current_twist.linear.x = 2.0
                            self.current_twist.angular.z = 0.0
                        elif key3 == 'B':
                            self.get_logger().info("Tortuga retrocediendo")
                            self.current_twist.linear.x = -2.0
                            self.current_twist.angular.z = 0.0
                        elif key3 == 'C':
                            self.get_logger().info("Tortuga girando a la derecha")
                            self.current_twist.linear.x = 0.0
                            self.current_twist.angular.z = -2.0
                        elif key3 == 'D':
                            self.get_logger().info("Tortuga girando a la izquierda")
                            self.current_twist.linear.x = 0.0
                            self.current_twist.angular.z = 2.0
                elif key == 's':
                    self.get_logger().info("Dibujando la letra 'S'")
                    self.teleport_to(4.0, 7.0, 3*math.pi/4)
                    time.sleep(0.3)
                    self.draw_s()
                elif key == 'a':
                    self.get_logger().info("Dibujando la letra 'A'")
                    self.teleport_to(4.5, 5.13, math.pi/3)
                    time.sleep(0.3)
                    self.draw_a()
                elif key == 'c':
                    self.get_logger().info("Dibujando la letra 'C'")
                    self.teleport_to(8.5, 7.0, 5*math.pi/6)
                    time.sleep(0.3)
                    self.draw_c()
                elif key == 'm':
                    self.get_logger().info("Dibujando la letra 'M'")
                    self.teleport_to(3.5, 4.5, math.pi/2)
                    time.sleep(0.3)
                    self.draw_m()
                elif key == 'b':
                    self.get_logger().info("Dibujando la letra 'B'")
                    self.teleport_to(5.0, 4.5, math.pi/2)
                    time.sleep(0.3)
                    self.draw_b()
                elif key == 'r':
                    self.get_logger().info("Reiniciando: limpiando pantalla y reubicando tortuga")
                    self.clear_background()
                    self.current_twist.linear.x = 0.0
                    self.current_twist.angular.z = 0.0
                    self.teleport_to(5.5, 5.5, 0.0)
                elif key == 'q':
                    self.get_logger().info("Saliendo")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def draw_s(self):
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 14.0
        start_time = time.time()
        while time.time() - start_time < 5*math.pi/(14*4)-0.05:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        time.sleep(0.2)
        self.teleport_to(3.65, 6.15, 0.0)
        time.sleep(0.2)
        self.current_twist.angular.z = -14.0
        self.current_twist.linear.x = 7.0
        mid_time = time.time()
        while time.time() - mid_time < 5*math.pi/(14*4):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_a(self):
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        rise_time = time.time()
        while time.time() - rise_time < 2.16/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -7.0
        mid_time = time.time()
        while time.time() - mid_time < 2*math.pi/(3*7):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        fall_time = time.time()
        while time.time() - fall_time < 2.11/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        time.sleep(0.2)
        self.teleport_to(5.0, 6.0, 0.0)
        time.sleep(0.2)
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        last_time = time.time()
        while time.time() - last_time < 0.97/7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_c(self):
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 7.0
        start_time = time.time()
        while time.time() - start_time < 4*math.pi/(7*3):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.05)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_m(self):
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t1 = time.time()
        while time.time() - t1 < 2.0 / 7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -7.0
        t2 = time.time()
        while time.time() - t2 < (math.pi / 4) * 3 / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)
        self.current_twist.angular.z = 0.0
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t3 = time.time()
        while time.time() - t3 < (math.pi / 4) / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 7.0
        t4 = time.time()
        while time.time() - t4 < (math.pi / 2) / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)
        self.current_twist.angular.z = 0.0
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t3 = time.time()
        while time.time() - t3 < (math.pi/4) / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 0.0             
        self.current_twist.angular.z = -7.0             
        t4 = time.time()
        while time.time() - t4 < (3 * math.pi / 4) / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)
        self.current_twist.angular.z = 0.0
        
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t1 = time.time()
        while time.time() - t1 < 2.0 / 7:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

    def draw_b(self):
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t1 = time.time()
        while time.time() - t1 < 3.0/7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = -7.0
        t2 = time.time()
        while time.time() - t2 < math.pi/(5.0):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
            
        self.teleport_to(5.0, 4.5 + 2.4, math.pi/6)  
        time.sleep(0.1) 
        
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        
        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = 0.0
        t3 = time.time()
        while time.time() - t3 < 0.5 / 7.0:
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 7.0
        self.current_twist.angular.z = -7.0
        t3 = time.time()
        while time.time() - t3 < math.pi/(5.0):
            self.publisher_.publish(self.current_twist)
            time.sleep(0.01)

        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

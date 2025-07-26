#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        
        # Declaración de parámetros para la comunicación serial
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(
                f"Puerto serial {port} abierto a {baud} baud.")
        except Exception as e:
            self.get_logger().error(
                f"Error al abrir puerto serial: {e}")
        
        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Broadcaster de transformaciones odom->base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Suscripción al tópico /cmd_vel para recibir comandos de velocidad
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Inicialización de variables para odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Parámetros físicos del robot (ajustar según tu configuración)
        self.wheel_radius = 0.035          # Radio de la rueda en metros
        self.ticks_per_revolution = 4532   # Pulsos por revolución del encoder (ajusta si es cuadratura, etc.)
        self.base_width = 0.145            # Distancia entre ruedas en metros
        
        # Variables para guardar el valor previo de ticks
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0
        
        # Timer para leer el puerto serial a ~50 Hz (cada 20ms)
        self.timer = self.create_timer(0.02, self.read_serial)
    
    def cmd_vel_callback(self, msg: Twist):
        # Extraer velocidades lineal (x) y angular (z)
        vx = msg.linear.x
        omega = msg.angular.z
        L = self.base_width
        
        # Cálculo de velocidades individuales de cada rueda
        speed_left = vx - omega * (L / 2)
        speed_right = vx + omega * (L / 2)
        
        # Conversión de velocidades a la unidad requerida por el ESP32.
        # Por ejemplo, si el ESP32 espera un valor PWM en el rango -255 a 255:
        k = 255 / 1.0  # Ajusta este factor según la máxima velocidad deseada
        pwm_left = int(max(min(speed_left * k, 255), -255))
        pwm_right = int(max(min(speed_right * k, 255), -255))
        
        # Preparar el comando en el formato "pwm_left,pwm_right\n"
        command = f"{pwm_left},{pwm_right}\n"
        self.get_logger().info(f"Enviando comando: {command.strip()}")
        if self.ser and self.ser.is_open:
            self.ser.write(command.encode())
        else:
            self.get_logger().warning("Puerto serial no disponible")
    
    def read_serial(self):
        try:
            if not (self.ser and self.ser.is_open):
                return
            # Leer una línea del puerto serial
            line = self.ser.readline().decode().strip()
            if line:
                # Se espera un formato "ticks_left,ticks_right" por línea
                ticks = line.split(',')
                if len(ticks) == 2:
                    ticks_left, ticks_right = int(ticks[0]), int(ticks[1])
                    
                    # Calcular diferencia de ticks desde la última lectura
                    delta_ticks_left = ticks_left - self.prev_ticks_left
                    delta_ticks_right = ticks_right - self.prev_ticks_right
                    self.prev_ticks_left = ticks_left
                    self.prev_ticks_right = ticks_right
                    
                    # Calcular la distancia recorrida por tick
                    distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
                    
                    # Distancia recorrida por cada rueda
                    dist_left = delta_ticks_left * distance_per_tick
                    dist_right = delta_ticks_right * distance_per_tick
                    
                    # Cálculo del desplazamiento lineal y angular
                    d = (dist_left + dist_right) / 2.0
                    dtheta = (dist_right - dist_left) / self.base_width
                    
                    # Actualización de la pose: integrar los incrementos
                    self.theta += dtheta
                    self.x += d * math.cos(self.theta)
                    self.y += d * math.sin(self.theta)
                    
                    # Preparar el mensaje de odometría
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = "odom"
                    odom_msg.child_frame_id = "base_link"
                    odom_msg.pose.pose.position.x = self.x
                    odom_msg.pose.pose.position.y = self.y
                    odom_msg.pose.pose.position.z = 0.0
                    
                    # Conversión de ángulo a cuaternión para la orientación
                    quat = quaternion_from_euler(0, 0, self.theta)
                    odom_msg.pose.pose.orientation.x = quat[0]
                    odom_msg.pose.pose.orientation.y = quat[1]
                    odom_msg.pose.pose.orientation.z = quat[2]
                    odom_msg.pose.pose.orientation.w = quat[3]
                    
                    # (Opcional) Se pueden incluir las velocidades calculadas en odom_msg.twist.twist
                    
                    # Publicar el mensaje de odometría
                    self.odom_pub.publish(odom_msg)
                    # Publicar la transformación TF odom -> base_link
                    t = TransformStamped()
                    t.header.stamp = odom_msg.header.stamp
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Aquí se puede incluir el broadcast de la transformación TF (odom → base_link)
                    # usando un TransformBroadcaster.
        except Exception as e:
            self.get_logger().error(f"Error en lectura serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

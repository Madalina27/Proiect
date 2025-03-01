import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import String

import math

def euler_from_quaternion(quaternion):
    """
    Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    """
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class RotateToAngleNode(Node):

    def __init__(self):
        super().__init__('rotate_to_angle')

        # Publisher pentru trimiterea comenzilor de miscare catre robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Subscriptie la topicul de odom a robotului pentru a citi dinamica lui
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        #Subscriptie la topicul care contine unghiul la care trebuie sa fie rotit robotul
        self.rotation_subscriber = self.create_subscription(Float32, '/target_angle', self.target_angle_callback, 10)

        #Publisher pentru a anunta finalizarea rotirii
        self.finished_publisher=self.create_publisher(String,'/finished_rotating',10)


        # Statusul robotului
        self.current_yaw = 0.0  # Orientare curenta (unghi de rotire in radiani)
        self.target_yaw = None  # Rotirea tinta (setata la apelul rotate_to_angle)
        self.angle_tolerance = 0.01  # Eroarea permisa (radiani)

        # Parametrii de control
        self.angular_speed = 0.5  # Viteza de rotatie (radiani/sec)

        # Timer pentru apelul regulat al loopului de control
        self.timer = self.create_timer(0.1, self.control_loop)

    

    def odom_callback(self, msg):
        """Callback pentru actualizarea axei de rotire din odometru"""
        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.current_yaw = euler_from_quaternion(quaternion)

    def target_angle_callback(self, msg):
        """Callback pentru obtinerea unghiului de la subscriber"""
        target_angle_deg = msg.data
        self.rotate_to_angle(target_angle_deg)


    def control_loop(self):
        """Loopul de control pentru rotirea robotului"""
        if self.target_yaw is None:
            # Fa nimic daca nu e setat niciun unghi de rotire
            return

        #Calculeaza cea mai mica diferenta de unghi pana la unghiul dorit
        angle_diff = self.normalize_angle(self.target_yaw - self.current_yaw)

        # Verifica daca ne aflam in limitele de toleranta pentru a finaliza rotatia
        if abs(angle_diff) <= self.angle_tolerance:
        
            self.stop_robot()
            self.duration=2
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds < (self.duration * 1e9):
                self.get_logger().info("Rotire robot pentru a atinge unghiul")
            self.get_logger().info("Unghiul dorit atins!")
            finished_msg=String()
            finished_msg.data='gata'
            self.finished_publisher.publish(finished_msg)
            self.target_yaw = None  # reseteaza unghiul pentru un nou apel
            return

        # Determinare directie de rotire
        twist = Twist()
        twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        self.cmd_vel_publisher.publish(twist)

    def normalize_angle(self, angle):
        """Normalizarea unghiului in intervalul [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def rotate_to_angle(self, target_angle_deg):
        """Setarea unghiului de rotatie (in grade) la care sa se roteasca robotul"""
        self.target_yaw = math.radians(target_angle_deg)
        self.get_logger().info(f"Rotirea spre {target_angle_deg} de grade...")

    
    def stop_robot(self):
        """Opreste miscarea robotului"""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    node = RotateToAngleNode()
    

    try:
        rclpy.spin(node)
      
    except KeyboardInterrupt:
        pass

    # Inchidere ROS 2
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

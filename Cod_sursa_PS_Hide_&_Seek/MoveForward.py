import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String


class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        
        # Subscriber la topicul care contine distanta care trebuie strabatuta
        self.subscription = self.create_subscription(
            Float32,
            '/move_forward_distance',
            self.distance_callback,
            10 
        )
        self.subscription  # Prevent unused variable warning

        # Publisher la topicul care controleaza miscarea robotului
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publisher pentru anuntarea finalizarii deplasarii robotului
        self.finished_publisher=self.create_publisher(String,'/finished_moving_forward',10)

    def distance_callback(self, msg):
        distance = msg.data
        self.get_logger().info(f'Distanta primita: {distance}')

        self.move_forward(distance)

    def move_forward(self, distance):
        
        velocity_msg = Twist()
        # Setare viteza liniara
        velocity_msg.linear.x = 0.5  # Ajustare viteza
        velocity_msg.angular.z = 0.0  # Sa nu existe rotatie

        # Calcularea duratei necesare de mers inainte cu o anumita viteza
        duration = distance / velocity_msg.linear.x

        self.get_logger().info(f'Merge inainte pentru {duration:.2f} secunde.')

        # Publicare comenzi de viteza pentru timpul calculat
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < (duration * 1e9):
            self.velocity_publisher.publish(velocity_msg)
            

        # Oprirea robotului
        velocity_msg.linear.x = 0.0
        
        for _ in range(10):  # Trimitem mesajul de stop de mai multe ori pentru a asigura oprirea
            self.velocity_publisher.publish(velocity_msg)
            self.get_logger().info('Oprire robot.')

        self.get_logger().info('Robot oprit.')
        finished_msg=String()
        finished_msg.data='gata'
        self.finished_publisher.publish(finished_msg)
        
            



def main(args=None):
    rclpy.init(args=args)

    distance_subscriber = MoveForwardNode()

    try:
        rclpy.spin(distance_subscriber)
    except KeyboardInterrupt:
        pass

    distance_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
from enum import Enum
from geometry_msgs.msg import Twist

ok_rotatie=0
ok_mers=0

#Enum pentru pozitiile fiecarui colt al cubului
class Pozitie(Enum):
    DREAPTA_JOS=1
    DREAPTA_SUS=2
    STANGA_SUS=3
    STANGA_JOS=4

#Enum pentru directia in care se uita robot la un anumit moment
class Directie(Enum):
    STANGA=1
    DREAPTA=2

#Nod folosit pentru semnalarea nodului principal atunci cand o rotire la un anumit unghi a fost finalizata
class VerificareRotatie(Node):
    def __init__(self):
        super().__init__('rotatie')
        
        my_callback_group = ReentrantCallbackGroup()
        
        #Subscriber pentru semnalarea finalizarii rotirii la un anumit unghi 
        self.finished_rotating_sub=self.create_subscription(
            String,
            '/finished_rotating',
            self.finished_rotating_callback,
            10,
            callback_group=my_callback_group
        )

    def finished_rotating_callback(self,msg):
        finalizare_rotatie=msg.data
        if(finalizare_rotatie=="gata"):
            global ok_rotatie
            ok_rotatie=1
            self.get_logger().info("rotatia din if")
        self.get_logger().info("call de la rotatie")


#Nod folosit pentru semnalarea nodului principal atunci cand deplasarea inainte pentru o anumita distanta a fost finalizata
class VerificareMersInainte(Node):
    def __init__(self):
        super().__init__('mers_inainte')
        my_callback_group = ReentrantCallbackGroup()
        #Subscriber pentru semnalarea finalizarii mersului inainte
        self.finished_moving_sub=self.create_subscription(
            String,
            '/finished_moving_forward',
            self.finished_moving_callback,
            10,
            callback_group=my_callback_group
        )
    def finished_moving_callback(self,msg):
        finalizare_mers=msg.data
        if(finalizare_mers=="gata"):
            global ok_mers
            ok_mers=1
            self.get_logger().info("mers inainte din if")
        self.get_logger().info("call de la mers inainte")
        
#Nod principal utilizat pentru detectarea culorilor
class ColorDetectorNode(Node):
    
    def __init__(self):
        super().__init__('color_detector')
        #Setare pozitii de start
        self.pozitie=Pozitie.DREAPTA_JOS
        self.directie=Directie.DREAPTA
       
        # Initializare CvBridge
        self.bridge = CvBridge()
        
        # Subscriber la topicul care contine imaginea captata de robot
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher pentru a trimite nodului de rotire unghiul la care sa o faca
        self.angle_pub = self.create_publisher(Float32, '/target_angle', 10)

        #Publisher pentru a trimite nodului de mers inainte distanta dorita
        self.move_forward_pub=self.create_publisher(Float32,'/move_forward_distance',10)

        #Publisher pentru a trimite topicului de control al robotului pentru al roti infinit
        self.rotate_infinitely_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    

    def image_callback(self, msg):
        # conversie din imagine ROS in imagine de tip OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        self.detect_colors(cv_image)



    def detect_colors(self, cv_image):
        # Conversia imaginii in spatiul de culori HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Definirea limitelor de culori in spatiul HSV (Verde si Rosu)
        green_lower = np.array([40, 40, 40])  # Limita inferioara pentru verde
        green_upper = np.array([80, 255, 255])  # Limita superioara pentru verde
        red_lower1 = np.array([0, 120, 70])  # Limita inferioara pentru rosu (primul interval)
        red_upper1 = np.array([10, 255, 255])  # ULimita superiora pentru rosu (primul interval)
        red_lower2 = np.array([170, 120, 70])  # Limita inferioara pentru rosu (al doilea interval)
        red_upper2 = np.array([180, 255, 255])  # Limita superioara pentru rosu (al doilea interval)

        #Crearea mastilor pentru detectia culorii verzi si rosii
        green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
        red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)

        # Combinarea celor 2 masti pentru detectia rosului
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Diferitele moduri de operare in functie de culoarea detectata
        if np.any(green_mask):
            self.get_logger().info("verde")

            if self.pozitie==Pozitie.DREAPTA_JOS:
                if self.directie==Directie.DREAPTA:

                    rotatie_msg=Float32()
                    rotatie_msg.data=90.

                    global ok_rotatie
                    ok_rotatie=0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie==0:
                        self.get_logger().info("Se roteste!")
                    

                    move_msg=Float32()
                    move_msg.data=6.1477

                    global ok_mers
                    ok_mers=0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers==0:
                        self.get_logger().info("Merge inainte!")
                    self.pozitie=Pozitie.STANGA_JOS

                else:
                    rotatie_msg=Float32()
                    rotatie_msg.data= 0.
                    
                    ok_rotatie=0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie==0:
                        self.get_logger().info("Se roteste!")
                    self.directie=Directie.DREAPTA


                    move_msg=Float32()
                    move_msg.data=6.1477
                   
                    ok_mers=0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers==0:
                        self.get_logger().info("Merge inainte!")
                    self.pozitie=Pozitie.DREAPTA_SUS

            elif self.pozitie == Pozitie.STANGA_JOS:
                if self.directie == Directie.STANGA:

                    rotatie_msg = Float32()
                    rotatie_msg.data = -90. 

                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
        
                    
                    move_msg = Float32()
                    move_msg.data = 6.1477

                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge inainte!")
                    self.pozitie = Pozitie.DREAPTA_JOS

                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data = 0. 

                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.STANGA


                    move_msg = Float32()
                    move_msg.data = 6.1477

                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge inainte!")
                    self.pozitie = Pozitie.STANGA_SUS


            elif self.pozitie == Pozitie.STANGA_SUS:
                if self.directie == Directie.DREAPTA:
                    
                    rotatie_msg = Float32()
                    rotatie_msg.data = 180.0  
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.STANGA 
        
                    
                    move_msg = Float32()
                    move_msg.data = 6.1477 
                    
                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge înainte!")
                    self.pozitie = Pozitie.STANGA_JOS

                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data = -89.9  
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")


                    move_msg = Float32()
                    move_msg.data = 6.1477 
                    
                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge înainte!")
                    self.pozitie = Pozitie.DREAPTA_SUS 

            elif self.pozitie == Pozitie.DREAPTA_SUS:
                if self.directie == Directie.DREAPTA:

                    rotatie_msg = Float32()
                    rotatie_msg.data = 89.8 
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!") 
                    
                   
                    move_msg = Float32()
                    move_msg.data = 6.1477  
                    
                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge înainte!")
                    self.pozitie = Pozitie.STANGA_SUS


                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data = 180.0 
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.DREAPTA 


                    move_msg = Float32()
                    move_msg.data = 6.1477  
                    
                    ok_mers = 0
                    self.move_forward_pub.publish(move_msg)
                    while ok_mers == 0:
                        self.get_logger().info("Merge înainte!")
                    self.pozitie = Pozitie.DREAPTA_JOS
            

        elif np.any(red_mask):
            self.get_logger().info("rosu")
            while 1:
                self.rotatie = Twist()
                self.rotatie.linear.x = 0.0 
                self.rotatie.angular.z = 2.  # Ajustare viteza de rotire(rad/s)
                self.rotate_infinitely_pub.publish(self.rotatie)

          
        else:
            self.get_logger().info("Nu s-a detectat verde sau rosu")

            if self.pozitie==Pozitie.DREAPTA_JOS:
                if self.directie==Directie.DREAPTA:

                    rotatie_msg=Float32()
                    rotatie_msg.data=90.
                   
                    ok_rotatie=0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie==0:
                        self.get_logger().info("Se roteste!")
                    self.directie=Directie.STANGA
                    
                else:
                    rotatie_msg=Float32()
                    rotatie_msg.data=0.
                    
                    ok_rotatie=0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie==0:
                        self.get_logger().info("Se roteste!")
                    self.directie=Directie.DREAPTA

                
            elif self.pozitie == Pozitie.STANGA_JOS:
                if self.directie == Directie.STANGA:

                    rotatie_msg = Float32()
                    rotatie_msg.data =-90. 

                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.DREAPTA
        
                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data =0. 

                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.STANGA


            elif self.pozitie == Pozitie.STANGA_SUS:
                if self.directie == Directie.DREAPTA:
                    
                    rotatie_msg = Float32()
                    rotatie_msg.data = 180.0  
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.STANGA 
        
                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data = -90.0  
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.DREAPTA

                    
            elif self.pozitie == Pozitie.DREAPTA_SUS:
                if self.directie == Directie.DREAPTA:

                    rotatie_msg = Float32()
                    rotatie_msg.data = 90.0 
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.STANGA  
                    
                else:
                    rotatie_msg = Float32()
                    rotatie_msg.data = 180.0 
                   
                    ok_rotatie = 0
                    self.angle_pub.publish(rotatie_msg)
                    while ok_rotatie == 0:
                        self.get_logger().info("Se roteste!")
                    self.directie = Directie.DREAPTA 
                      

def main(args=None):
    rclpy.init(args=args)

    node = ColorDetectorNode()
    node2=VerificareMersInainte()
    node3=VerificareRotatie()
    
    # activarea rularii in paralel a celor 3 noduri
    executor=MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)
    executor.add_node(node3)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
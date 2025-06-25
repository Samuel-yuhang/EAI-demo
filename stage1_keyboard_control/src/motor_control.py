import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from gpiozero import Motor 
from time import sleep  

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')  
        self.get_logger().info('Test node has been started')  
        

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10  
        )

        self.motors = {
            "right_front": Motor(forward=17, backward=27),
            "left_front": Motor(forward=5, backward=6),
            "right_back": Motor(forward=13, backward=19),
            "left_back": Motor(forward=12, backward=16)
        }


        self.motors_target_speed = {
            "right_front": 0, "left_front": 0, "right_back": 0, "left_back": 0
        }

    def listener_callback(self, msg):
        linear_x = msg.linear.x  
        linear_y = msg.linear.y  
        angular_z = msg.angular.z  
        
        self.motors_target_speed["left_front"] = (linear_x - linear_y - angular_z)
        self.motors_target_speed["right_front"] = (linear_x + linear_y + angular_z)
        self.motors_target_speed["left_back"] = (linear_x + linear_y - angular_z)
        self.motors_target_speed["right_back"] = (linear_x - linear_y + angular_z)
        
        for wheel in self.motors.keys():
            if self.motors_target_speed[wheel] > 1 or self.motors_target_speed[wheel] < -1:
                self.get_logger().info(f"Wheel {wheel} speed is out of limit: {self.motors_target_speed[wheel]}")
            self.motors_target_speed[wheel] = max(min(self.motors_target_speed[wheel], 1), -1)
        
        for wheel in self.motors.keys():
            if self.motors_target_speed[wheel] > 0:
                self.motors[wheel].forward(speed=self.motors_target_speed[wheel])
            elif self.motors_target_speed[wheel] < 0:
                self.motors[wheel].backward(speed=-self.motors_target_speed[wheel])
            else:
                self.motors[wheel].stop()

def main(args=None):
    rclpy.init(args=args) 
    test_node = TestNode() 
    rclpy.spin(test_node) 
    test_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__':
    main() 
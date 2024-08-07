import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class SpotControl(Node):
    def __init__(self):
        super().__init__('SpotControl')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.body_pub = self.create_publisher(String, 'body_pose', 1)

        self.current_mode = " "
        self.linear_x_scale = 1.5
        self.angular_scale = 0.8
        feq = 10
        self.joy = None
        self.cmd = None
        
        self.timer = self.create_timer(1/feq, self.timer_callback)
        
    def cmd_callback(self, data: Twist):
        self.cmd = data
        
    def joy_callback(self, data: Joy):
        # Logi F710 D mode and controls
        trig = Trigger.Request()
        # self.get_logger().info('current_mode: {}'.format(self.current_mode))
        if(data.buttons[3]==1 and self.current_mode != "y"):
            self.current_mode = "y"
            time.sleep(0.1)
        elif(data.buttons[0]==1 and self.current_mode != "a"):
            self.current_mode = "a"
            time.sleep(0.1)            
        elif(data.buttons[2]==1):
            self.current_mode = "x"
            time.sleep(0.1)
        elif(data.buttons[1]==1 and self.current_mode != "b"):
            self.current_mode = "b"
            time.sleep(0.1)
        elif(data.buttons[7]==1 and self.current_mode != "start"):
            self.current_mode = "start"
            time.sleep(0.1)
        elif(data.axes[7]==1 and self.current_mode != "up"):
            self.current_mode = "up"
            time.sleep(0.1)
        elif(data.axes[7]==-1 and self.current_mode != "down"):
            self.current_mode = "down"
            time.sleep(0.1)
        elif(data.axes[6]==-1 and self.current_mode != "left"):
            self.current_mode = "left"
            time.sleep(0.1)
        elif(data.axes[6]==1 and self.current_mode != "right"):
            self.current_mode = "right"
            time.sleep(0.1)
        elif(data.axes[5]==-1 and data.axes[2]==-1):
            self.current_mode = "stop"
            time.sleep(0.1)
        else:
            self.current_mode = " "
        self.joy = data
    def timer_callback(self):
        vel = Twist()
        pose = String()
        #print(type(self.current_mode))
        pose.data=str(self.current_mode)
        if self.joy is not None:
            # Now safe to access self.joy.axes
            vel.linear.x = self.joy.axes[4]*self.linear_x_scale
            vel.angular.z = self.joy.axes[3]*self.angular_scale
        #pose=self.current_mode
        self.cmd_pub.publish(vel)
        self.body_pub.publish(pose)
        
def main(args=None):
    rclpy.init(args=args)

    controller = SpotControl()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

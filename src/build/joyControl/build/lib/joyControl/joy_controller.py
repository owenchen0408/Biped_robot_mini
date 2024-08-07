import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Char



class SpotControl(Node):
    def __init__(self):
        super().__init__('SpotControl')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.body_pub = self.create_publisher(Char, 'body_pose', 1)

        self.current_mode = 0
        self.linear_x_scale = 1.5
        self.angular_scale = 0.8
        feq = 20
        self.joy = None
        self.cmd = None
        
        self.timer = self.create_timer(1/feq, self.timer_callback)
        
    def cmd_callback(self, data: Twist):
        self.cmd = data
        
    def joy_callback(self, data: Joy):
        # Logi F710 D mode
        trig = Trigger.Request()
        # self.get_logger().info('current_mode: {}'.format(self.current_mode))
        if(data.buttons[1]==1):
            self.current_mode = 1
        elif(data.buttons[0]==1):
            self.current_mode = 2
        self.joy = data
        
    def timer_callback(self):
        vel = Twist()
        pose = Char()
        pose.data=self.current_mode
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

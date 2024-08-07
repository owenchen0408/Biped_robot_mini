import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from spot_msgs.srv import SetLocomotion
from geometry_msgs.msg import Twist, Pose
from tf_transformations import quaternion_from_euler

class SpotControl(Node):
    def __init__(self):
        super().__init__('SpotControl')
        self.create_subscription(Joy, "joy", self.joy_callback, 1)
        self.create_subscription(Twist, "auto_cmd_vel", self.cmd_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.body_pub = self.create_publisher(Pose, 'body_pose', 1)

        self.current_mode = ''
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
        if data.buttons[4] == 1 and data.buttons[5] == 1 and data.buttons[2] == 1: # LB + RB + B
            self.current_mode = 'Stop'
            # Power
            self.estop_client.call_async(trig) # Estop
            return
        if data.buttons[6] == 1 and data.buttons[7] == 1 and data.buttons[2] == 1: # LT + RT + B
            self.current_mode = ''
            # Power
            self.estop_release_client.call_async(trig) # Estop Release
            self.power_on_client.call_async(trig) # Power On
            return
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[0] == 1:
            srv = SetLocomotion.Request()
            srv.locomotion_mode = 8
            self.current_mode = 'Hop'
            self.locomotion_client.call_async(srv) # Hop
            return
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[1] == 1:
            srv = SetLocomotion.Request()
            srv.locomotion_mode = 5
            self.current_mode = 'Amble'
            self.locomotion_client.call_async(srv) # Amble
            return
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[2] == 1:
            srv = SetLocomotion.Request()
            srv.locomotion_mode = 4
            self.current_mode = 'Crawl'
            self.locomotion_client.call_async(srv) # Crawl
            return
        if data.buttons[4] == 1 and data.buttons[5] != 1 and data.buttons[3] == 1:
            srv = SetLocomotion.Request()
            srv.locomotion_mode = 7
            self.current_mode = 'Jog'
            self.locomotion_client.call_async(srv) # Jog
            return
        if data.buttons[4] != 1 and data.buttons[0] == 1:
            self.current_mode = 'Sit'
            self.sit_client.call_async(trig) # Sit
            return
        if data.buttons[4] != 1 and data.buttons[1] == 1:
            srv = SetLocomotion.Request()
            srv.locomotion_mode = 1
            self.current_mode = 'Walk'
            self.locomotion_client.call_async(srv) # Walk
            return
        if data.buttons[4] != 1 and data.buttons[2] == 1:
            self.current_mode = 'Stand'
            self.stand_client.call_async(trig) # Stand
            return
        if data.buttons[9] == 1: # start btn
            self.current_mode = 'Auto'
            return
        
        self.joy = data
        
    def timer_callback(self):
        if self.current_mode == 'Hop' or self.current_mode == 'Jog' \
            or self.current_mode == 'Amble' or self.current_mode == 'Crawl' \
            or self.current_mode == 'Walk' or self.current_mode == 'Stair':
            # cmd_vel
            vel = Twist()
            vel.linear.x = self.joy.axes[1]*self.linear_x_scale
            vel.angular.z = self.joy.axes[2]*self.angular_scale
            self.cmd_pub.publish(vel)
        
        elif self.current_mode == 'Auto':
            vel = Twist()
            if self.cmd:
                vel = self.cmd
            self.cmd_pub.publish(vel)
            
        elif self.current_mode == 'Stand':
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = self.joy.axes[1]*0.2
            roll, pitch, yaw = -self.joy.axes[0]*0.6, -self.joy.axes[3]*0.6, self.joy.axes[2]*0.6
            q = quaternion_from_euler(roll, pitch, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.body_pub.publish(pose)
        
def main(args=None):
    rclpy.init(args=args)

    controller = SpotControl()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

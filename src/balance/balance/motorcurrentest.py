from module.foc_motor_serial import MotorControl
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

mc = MotorControl(device_name="/dev/foc",baudrate=1000000)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher( Int32 , 'current', 10)  
        self.i = 0

    def current_pub(self,data):
        msg = Int32()
        msg.data = data
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        

try:
    rclpy.init(args=None)
    currentpub = MinimalPublisher()

    i = 0 
    for _ in range(20):
        i+=100

        print("torque:",i)
        for k in range(300*1):
            a = mc.torquecontrol(0x01, i)
            
            if a != None:
                print(a[4] , i , k)
                
                currentpub.current_pub(a[4])
            else:
                continue
   
    mc.stopmotor(0x01)
    
    time.sleep(1)
    mc.closeport()


except KeyboardInterrupt:
        
        mc.stopmotor(0x01)
        mc.stopmotor(0x02)
        time.sleep(1)
        mc.closeport()

                


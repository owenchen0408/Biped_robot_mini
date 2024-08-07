import rclpy
import math
import time
import sys
import os
import numpy as np
from rclpy.node import Node
from simple_pid import PID
from sensor_msgs.msg import Imu
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from module.DXL_motor_control import DXL_Conmunication
from module.foc_motor_serial import MotorControl

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(Imu,'imu/data_raw' , self.listener_callback, 1)
        self.subscription

    def listener_callback(self, msg):
        
            qua_x = msg.orientation.x
            qua_y = msg.orientation.y
            qua_z = msg.orientation.z
            qua_w = msg.orientation.w
            # self.pitch_degree =math.degrees(math.atan2(2*(qua_w*qua_z+qua_x*qua_y),1-2*(qua_y*qua_y+qua_z*qua_z)))
            self.pitch_init= math.asin(2 * (qua_w * qua_y - qua_z * qua_x))

            self.pitch_degree , _ =  self.complementary_filter(msg.angular_velocity.x    , msg.angular_velocity.y   , msg.angular_velocity.z,
                                                               msg.linear_acceleration.x , msg.linear_acceleration.y, msg.linear_acceleration.z,
                                                               0 , 0 , 0.98 , 0.005)
            
    def returndegree(self):

        # if (self.pitch_degree > 0):
        #     self.pitch_degree = 180 - self.pitch_degree
        # elif (self.pitch_degree < 0):
        #     self.pitch_degree = -(180 + self.pitch_degree)

        return self.pitch_degree
    def get_accelerometer_angles(self, ax, ay, az):
        pitch_acc = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        roll_acc = np.arctan2(-ax, az)
        return pitch_acc, roll_acc
    
    def complementary_filter(self,ax, ay, az, gx, gy, gz, pitch, roll, alpha, dt):
    # 使用加速度計計算的角度
        pitch_acc, roll_acc = self.get_accelerometer_angles(ax, ay, az)
        
        # 使用陀螺儀計算的角度
        pitch_gyro = pitch + gx * dt
        roll_gyro = roll + gy * dt
        
        # 互補濾波器融合
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
        roll = alpha * roll_gyro + (1 - alpha) * roll_acc
        print('Pitch:',math.degrees(pitch) ,'\nRoll:',roll)
        return pitch, roll

def main():
    rclpy.init()
    imu_subscriber = ImuSubscriber()
    try:
         while rclpy.ok():
              rclpy.spin_once(imu_subscriber)
    except KeyboardInterrupt:
        print("Shutting down gracefully.")
        rclpy.shutdown()

if __name__ =='__main__':
    main()
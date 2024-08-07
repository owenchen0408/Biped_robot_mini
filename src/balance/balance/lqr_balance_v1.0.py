import rclpy
import math
import time
import sys
import os
import numpy as np
from rclpy.node import Node
from simple_pid import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from module.DXL_motor_control import DXL_Conmunication
from module.foc_motor_serial import MotorControl
import matplotlib.pyplot as plt

from LQR import InvertedPendulumLQR

Frequency = 80

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.motor_control = robotmotor('motor01','motor02','motor11','motor12','wheel1','wheel2')
        # self.pid_controller = PID_controller()
        self.imu_subscriber = ImuSubscriber()
        self.lqr_controller = InvertedPendulumLQR()
        
        # self.X_dot_pub = self.create_publisher(Float32 , 'X_dot' , 1)
        self.X_arr_pub = self.create_publisher(Float32MultiArray, 'X_arr', 1)
        self.u_pub = self.create_publisher(Float32 , 'u' , 1)
        

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        global Frequency
        self.subscription = self.create_subscription(Imu,'imu/data_raw' , self.listener_callback, 1)
        self.theta_pub = self.create_publisher(Float32 , 'theta' , 1)
        self.pitch_middle = 0
        self.pitch_bias = 0
        self.frequency = Frequency      
        self.pitch_vel = 0

    def listener_callback(self, msg):
        qua_x = msg.orientation.x
        qua_y = msg.orientation.y
        qua_z = msg.orientation.z
        qua_w = msg.orientation.w
        # self.pitch_degree =math.degrees(math.atan2(2*(qua_w*qua_z+qua_x*qua_y),1-2*(qua_y*qua_y+qua_z*qua_z)))
        self.pitch = (math.asin(2 * (qua_w * qua_y - qua_z * qua_x)))*(180/math.pi)+2

        self.pitch_vel = msg.angular_velocity.y
        print('pitch', -self.pitch, -self.pitch_vel)
        # self.pitch_middle = self.pitch_middle - self.pitch_bias / self.frequency * 0.1
        # self.pitch_bias = self.pitch_middle - self.pitch
        # self.pitch_least = 0
        # self.pitch_degree , _ =  self.complementary_filter(msg.augular_velocity.x    , msg.augular_velocity.y   , msg.augular_velocity.z,
        #                                                    msg.linear_acceleration.x , msg.linear_acceleration.y, msg.linear_acceleration.z,
        #                                                    0 , 0 , 0.98 , 0.005)
            
    def returndegree(self):
        theta_msg = Float32()
    #     # if (self.pitch_degree > 0):
    #     #     self.pitch_degree = 180 - self.pitch_degree
    #     # elif (self.pitch_degree < 0):
    #     #     self.pitch_degree = -(180 + self.pitch_degree)
        # self.pitch_init *= 0.7
        # self.pitch_init += self.pitch_least*0.3
        # self.pitch_least = self.pitch_init
        theta_msg.data = float(-self.pitch)
        # print(theta_msg.data)
        # print(self.pitch_init)
        self.theta_pub.publish(theta_msg)
        
        return -self.pitch, -self.pitch_vel
    # def get_accelerometer_angles(self, ax, ay, az):
    #     pitch_acc = np.arctan2(ay, np.sqrt(ax**2 + az**2))
    #     roll_acc = np.arctan2(-ax, az)
    #     return pitch_acc, roll_acc
    
    # def complementary_filter(self,ax, ay, az, gx, gy, gz, pitch, roll, alpha, dt):
    # # 使用加速度計計算的角度
    #     pitch_acc, roll_acc = self.get_accelerometer_angles(ax, ay, az)
        
    #     # 使用陀螺儀計算的角度
    #     pitch_gyro = pitch + gx * dt
    #     roll_gyro = roll + gy * dt
        
    #     # 互補濾波器融合
    #     pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    #     roll = alpha * roll_gyro + (1 - alpha) * roll_acc
        
    #     return pitch, roll
    
class robotmotor:
    
    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2):

        self.dxl = DXL_Conmunication(device_name="/dev/dxl",b_rate=57600)
        self.mc = MotorControl(device_name="/dev/foc",baudrate=1000000)
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11  
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.createdxlmotor()
        # self.balance_pid = PID(balance_kp, balance_ki ,balance_kd,setpoint=balance_setpoint)
        # self.balance_pid.output_limits = (-4000,4000)
        # # self.balance_pid.output_limits = (-2000,2000)
        # self.velocity_pid = PID(velocity_kp,velocity_ki, 0 ,setpoint=velocity_setpoint)
        # self.velocity_pid.output_limits = (-4000,4000)
        # # self.balance_pid.output_limits = (-2000,2000)
        

    def createdxlmotor(self):

        self.motor01 = self.dxl.createMotor("motor01", 1)
        self.motor02 = self.dxl.createMotor("motor02", 2)
        self.motor11 = self.dxl.createMotor("motor11", 11)
        self.motor12 = self.dxl.createMotor("motor12", 12)
        self.dxl.addAllBuckPrarmeter()
        self.motor01.enableMotor()
        self.motor02.enableMotor()
        self.motor11.enableMotor()
        self.motor12.enableMotor()
        self.dxl.updateMotorData()

    def startfocmotor(self):

        self.mc.startmotor(0x01)
        self.mc.startmotor(0x02)

    def lockleg(self, angle01=2514, angle02=3530):
        
        self.dxl.updateMotorData()
        self.motor01.writePosition(angle01)
        self.motor02.writePosition(angle02)
        self.motor11.writePosition(angle01)
        self.motor12.writePosition(angle02)
        self.dxl.sentAllCmd()


    def motortorquecommand(self,id,torque):
        # if abs(torque) >= 0.33:
            # a = self.mc.torquecontrol(id ,torque)
        a = self.mc.torquecontrol(id ,torque)
        
        return a
        
    def motorspeedcommand(self, id, speed):
        a = self.mc.speedcontrol(id ,speed)

        return a
        
    def disableALLmotor(self):
        self.dxl.disableAllMotor()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)
        time.sleep(1)
        self.dxl.closeHandler()
        self.mc.closeport()
    
    # def balancepid(self, degree):
    #     output1 = self.balance_pid(degree)
    #     return(output1)
    
    # def velocitypid(self, motor_rpm):

    #     output2 = self.velocity_pid(motor_rpm)
    #     return(output2)

# def int_to_float(value):
#     # Convert the integer to binary and remove the '0b' prefix
#     binary_str = bin(value)[2:]
    
#     # If the value is negative, bin() returns the two's complement form, so we need to handle it
#     if value < 0:
#         # Calculate the two's complement for negative values
#         binary_str = bin((1 << 16) + value)[2:]
    
#     binary_str = binary_str.zfill(16) if value >= 0 else binary_str[-16:]
    
#     int_value = 0
#     # Check if the binary string represents a negative number
#     if binary_str[0] == '1':  # Negative number
#         # Invert the bits
#         inverted = ''.join('1' if b == '0' else '0' for b in binary_str)
#         # Convert inverted binary to decimal and add 1
#         int_value = -int(inverted, 2) + 1
#     else:  # Positive number
#         # Directly convert to decimal
#         int_value = int(binary_str, 2)

#     float_value = float(int_value)
    
#     return float_value

def main():
    global Frequency
    rclpy.init()
    controller = Controller()
    # imu_subscriber = ImuSubscriber()
    robot_motor = robotmotor('motor01','motor02','motor11','motor12','wheel1','wheel2')
    angle01=2514
    angle02=3530
    controller.motor_control.lockleg(angle01=angle01,angle02=angle02)
    u_msg = Float32()
    X_arr_msg = Float32MultiArray()
    time.sleep(5)
    hip = (angle02-2706)/4096*360
    knee = (angle01-1760)*81/1000+64
    controller.lqr_controller.__init__(hip, knee, delta_t=1/Frequency)
    pitch = 0.0
    pitch_dot = 0.0
    x_dot = 0.0
    x_dot_last = 0
    X = np.array([
            [0.0],
            [0.0],
            [0.0],
            [0.0]
        ])
    try:
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(controller.imu_subscriber) 
            pitch_last = pitch
            pitch, pitch_dot_imu = controller.imu_subscriber.returndegree()
            pitch = math.radians(pitch)
            # pitch_dot = math.radians(pitch_dot)
            # print(pitch_middle)
            # robot_motor.balance_pid = PID(balance_kp, balance_ki ,balance_kd,setpoint=pitch_middle)
            
            if  -40 <=  math.degrees(pitch) <= 40:

                # print(pitch)
                # output1 = controller.pid_controller.balancepid(pitch)
                # output2 = robot_motor.velocitypid(speed)
                # calculate control input
                u = controller.lqr_controller.lqr_control(X)
                if u == None:
                    continue
                # print(u,'N')
                # print(X[0],'\n')
                # simulate inverted pendulum cart
                u_msg.data = float(u)
                print("u:", u)
                if u[0, 0] > 0.142:
                    u[0, 0] = 0.142
                elif u[0, 0] < -0.142:
                    u[0, 0] = -0.142
                outputall = u[0, 0] * 300/0.142/2.65 #*(0.8**abs(pitch*0.2)) 364
                outputall = int(outputall)
                # print("output:", outputall)
                # a1 = controller.motor_control.motorspeedcommand(0x01, outputall)
                # a2 = controller.motor_control.motorspeedcommand(0x02, -outputall)
                a1 = controller.motor_control.motortorquecommand(0x01, -outputall)
                a2 = controller.motor_control.motortorquecommand(0x02, outputall)

                if a1 is None or a2 is None:
                    continue
                x_dot_last = x_dot
                x_dot = float(a2[2])*2*np.pi/60
                # print(a2[4])
            else:
                # a1 = controller.motor_control.motorspeedcommand(0x01, 0)
                # a2 = controller.motor_control.motorspeedcommand(0x02, 0)
                a1 = controller.motor_control.motortorquecommand(0x01, 0)
                a2 = controller.motor_control.motortorquecommand(0x02, 0)


            if a1 is None or a2 is None:
                continue
            t1 = time.time()
            print('freq', 1/(t1-t0))
            x = (x_dot - x_dot_last) * (t1-t0)
            pitch_dot_new = (pitch-pitch_last) / (t1-t0)
            if abs(pitch_dot_new) < 1.57:
                pitch_dot = pitch_dot_new
            t0 = t1

            X = np.array([
                        [x],
                        [x_dot],
                        [pitch],
                        [pitch_dot]
                    ])
            X_arr_msg.data = [float(x), float(x_dot), float(pitch), float(pitch_dot)]
            print('pitch_dot', pitch_dot, pitch_dot_imu)
            print("X", X)
            # print(x_dot_msg.data)
            controller.X_arr_pub.publish(X_arr_msg)
            controller.u_pub.publish(u_msg)

    except KeyboardInterrupt:

        print("Shutting down gracefully.")
        controller.motor_control.disableALLmotor()
        rclpy.shutdown()
        

    finally:
        controller.motor_control.disableALLmotor()
        rclpy.shutdown()
        

if __name__ == '__main__':

    main()
    
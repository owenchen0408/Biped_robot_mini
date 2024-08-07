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
import threading

from LQR import InvertedPendulumLQR

Frequency = 80

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        angle01=2514
        angle02=3530
        hip = (angle02-2706)/4096*360
        knee = (angle01-1760)*81/1000+64
        
        # controller.lqr_controller.__init__(hip, knee, delta_t=1/Frequency)
        self.motor_control = robotmotor('motor01','motor02','motor11','motor12','wheel1','wheel2')
        self.motor_control.lockleg(angle01=angle01,angle02=angle02)
        self.imu_subscriber = ImuSubscriber()
        self.lqr_controller = InvertedPendulumLQR(hip, knee, delta_t=1/200)
        self.imu_thread = threading.Thread(target=rclpy.spin, args=(self.imu_subscriber,), daemon=True)
        self.imu_thread.start()
        print('imu thread activate')
        self.motor_thread = threading.Thread(target=self.motor_control.motor_loop, args=(), daemon=True)
        self.motor_thread.start()
        print('motor thread activate')
        self.update_thread = threading.Thread(target=self.update_u, args=(), daemon=True)
        time.sleep(2)
        self.run_flag = True
        self.update_thread.start()
        print('update thread activate')

        self.X_arr_pub = self.create_publisher(Float32MultiArray, 'X_arr', 1)
        self.u_pub = self.create_publisher(Float32 , 'u' , 1)
        
    
    def closeSystem(self):
        self.motor_control.run_flag = False
        self.run_flag = False
        self.motor_thread.join()
        self.motor_control.disableALLmotor()


    def update_u(self):
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
        t0 = time.time()
        X_arr_msg = Float32MultiArray()
        u_msg = Float32()
        while self.run_flag:
            pitch_last = pitch
            pitch, pitch_dot_imu = self.imu_subscriber.returndegree()
            if  abs(pitch) > 40:
                self.motor_control.u[0, 0] = 0.0
                continue
            pitch = math.radians(pitch)
            a1, a2 = self.motor_control.get_motor_feedback()
            if a1 is None or a2 is None:
                continue
            x_dot_last = x_dot
            x_dot = float(a2[2])*2*np.pi/60
            t1 = time.time()
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
            self.motor_control.u = self.lqr_controller.lqr_control(X)
            u_msg.data = float(self.motor_control.u[0,0])
            X_arr_msg.data = [float(x), float(x_dot), float(pitch), float(pitch_dot)]
            print("X", X)
            self.X_arr_pub.publish(X_arr_msg)
            self.u_pub.publish(u_msg)
            # time.sleep(1/200)


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
        self.pitch = (math.asin(2 * (qua_w * qua_y - qua_z * qua_x)))*(180/math.pi)+1.5

        self.pitch_vel = msg.angular_velocity.y
        print('pitch', -self.pitch, -self.pitch_vel)                        
            
    def returndegree(self):
        theta_msg = Float32()
        theta_msg.data = float(-self.pitch)
        self.theta_pub.publish(theta_msg)
        
        return -self.pitch, -self.pitch_vel
    

    
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
        self.a1 = []
        self.a2 = []
        self.u = np.zeros((1,1))
        self.createdxlmotor()
        self.run_flag = True
        

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

    def motor_loop(self):
        while self.run_flag:
            if self.u[0, 0] > 0.142:
                self.u[0, 0] = 0.142
            elif self.u[0, 0] < -0.142:
                self.u[0, 0] = -0.142
            outputall = self.u[0, 0] * 300/0.142/2.65 #*(0.8**abs(pitch*0.2)) 364
            outputall = int(outputall)
            self.a1 = self.motortorquecommand(0x01, -outputall)
            self.a2 = self.motortorquecommand(0x02, outputall)

    def get_motor_feedback(self):
        return self.a1, self.a2

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


def main():
        global Frequency
        rclpy.init()
        controller = Controller()
        while True:
            try:
                pass
            except KeyboardInterrupt:
                controller.closeSystem()
                rclpy.shutdown()
                break
            finally:
                controller.closeSystem()
                rclpy.shutdown()
                break
        


    

    # finally:
    #     controller.motor_control.disableALLmotor()
    #     rclpy.shutdown()
        

if __name__ == '__main__':

    main()
    
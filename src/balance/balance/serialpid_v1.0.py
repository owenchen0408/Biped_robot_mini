from module.foc_motor_serial import MotorControl
from module.DXL_motor_control import DXL_Conmunication
import rclpy
import math
import time
import sys
import os
import threading
import numpy as np
from rclpy.node import Node
# from simple_pid import PID
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import traceback
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# balance_kp = 103*0.6
# balance_ki = 2*balance_kp/0.5
# balance_kd = 0.125*103*0.5
angularv_kp = 30
angularv_ki = 10

balance_kp = 0  # *0.75
balance_kd = 0  # 0.5
COM_bias = 0

velocity_kp = 0
velocity_ki = 0
velocity_setpoint = 0

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, target, now, dt):

        error = target - now
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        
        return output
    
    # def output_limits(self,min,max):

    #     self.max = max
    #     self.min = min



class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imusubscriber')
        self.subscription = self.create_subscription(
            Imu, 'imu/data_raw', self.listener_callback, 1)
        # self.theta_Pub = self.create_publisher(Float32 , 'theta' , 1)
        # self.X_dot_Pub = self.create_publisher(Float32 , 'X_dot' , 1)
        self.pitch_init = 0
        self.angularvelocity_y = 0

    def listener_callback(self, msg):

        qua_x = msg.orientation.x
        qua_y = msg.orientation.y
        qua_z = msg.orientation.z
        qua_w = msg.orientation.w
        self.angularvelocity_y = msg.angular_velocity.y
        self.pitch_init = (
            math.asin(2 * (qua_w * qua_y - qua_z * qua_x)))*(180/math.pi)

    def getImuOrientation(self) -> float:
        return -np.deg2rad(self.pitch_init)


class robotcontrol:

    def __init__(self, motor01, motor02, motor11, motor12, wheel1, wheel2):
        rclpy.init()
        self.dxl = DXL_Conmunication(device_name="/dev/dxl", b_rate=57600)
        self.mc = MotorControl(device_name="/dev/foc", baudrate=1000000)
        self.imu_node = ImuSubscriber()
        imu_thread = threading.Thread(
            target=rclpy.spin, args=(self.imu_node,), daemon=True)
        imu_thread.start()
        self.pid_thread = None
        self.motor01 = motor01
        self.motor02 = motor02
        self.motor11 = motor11
        self.motor12 = motor12
        self.wheel1 = wheel1
        self.wheel2 = wheel2
        self.createdxlmotor()

        self.balance_pid = PID(10, 0, 1)
        # self.balance_pid.output_limits = (-10, 10)
        self.velocity_pid = PID(0.006, 0.0, 0.000001)
        # self.velocity_pid.output_limits = (0, 0)
        self.angularvelocity_pid = PID(65, 0, 0)
        # self.angularvelocity_pid.output_limits = (-300, 300)

        self.dt = 1 / 200
        self.prev_pitch = 0
        self.isRunning = False

    def getControllerPIDParam(self):
        print(f'Balance kp: {self.balance_pid.Kp}, kd: {self.balance_pid.Kd}')
        print(f'Angular kp: {self.angularvelocity_pid.Kp}, ki: {self.angularvelocity_pid.Ki}')
        print(f'Velocity kp: {self.velocity_pid.Kp}, ki: {self.velocity_pid.Ki}')
        return self.balance_pid.Kp, self.balance_pid.Kd, self.angularvelocity_pid.Kp, self.angularvelocity_pid.Kd

    def setAngularPI(self, kp, ki):
        self.angularvelocity_pid.Kp = kp
        self.angularvelocity_pid.Ki = ki
    
    def setBalancePD(self, kp, kd):
        self.balance_pid.Kp = kp
        self.balance_pid.Kd = kd
    
    def setVelocityPID(self, kp, kd):
        self.velocity_pid.Kp = kp
        self.velocity_pid.Kd = kd

    def createdxlmotor(self):

        self.motor01 = self.dxl.createMotor("motor01", 1)
        self.motor02 = self.dxl.createMotor("motor02", 2)
        self.motor11 = self.dxl.createMotor("motor11", 11)
        self.motor12 = self.dxl.createMotor("motor12", 12)
        self.dxl.addAllBuckPrarmeter()
        self.enableAllMotor()
        self.dxl.updateMotorData()

    def enableAllMotor(self):
        self.motor01.enableMotor()
        self.motor02.enableMotor()
        self.motor11.enableMotor()
        self.motor12.enableMotor()

    def startfocmotor(self):

        self.mc.startmotor(0x01)
        self.mc.startmotor(0x02)

    def lockleg(self):

        self.enableAllMotor()
        self.dxl.updateMotorData()
        self.motor01.writePosition(2760)  # 2760-1795
        self.motor02.writePosition(3795)  # 4095-2900
        self.motor11.writePosition(2760)
        self.motor12.writePosition(3795)
        self.dxl.sentAllCmd()

    def motortorquecommand(self, id, torque):
        a = self.mc.torquecontrol(id, torque)

        return a

    def motorspeedcommand(self, id, speed):
        a = self.mc.speedcontrol(id, speed)

        return a

    def disableALLmotor(self):
        self.isRunning = False
        if self.pid_thread is not None:
            self.pid_thread.join()
        self.dxl.disableAllMotor()
        self.mc.stopmotor(0x01)
        self.mc.stopmotor(0x02)

    def closeSystem(self):
        self.dxl.closeHandler()
        self.mc.closeport()
    
    def getPitchDot(self, pitch):
        pitch_dot = (pitch - self.prev_pitch) / self.dt
        self.prev_pitch = pitch
        return pitch_dot
            


    def controller(self):
        
        dt = 1 / 200
        motorrpm = 0
        desire_pitch = 0
        position = 0.0
        middle_ang = -0.03 #-0.028
        desire_vel = 0
        desire_pos = 0
        motor_speed = 0.0
        while True:
            start = time.time()
            if not self.isRunning:
                break
            pitch = self.imu_node.getImuOrientation()
            print(f'pitch : {pitch}')
            pitch_velocity = self.getPitchDot(pitch)
            # print(f'pitch vel: {pitch_velocity}')

            ## Adapt COG offset angle ##
            # angle_bias = desire_pitch - pitch
            output2 = self.balance_pid.update(desire_pitch, pitch,dt)
            # output2 = self.balance_pid.update(desire_pitch, pitch,dt)
            # desire_pitch -= angle_bias * 0.00001
            # print(f'dersire ang {desire_pitch}')
            output3 = self.angularvelocity_pid.update(output2, pitch_velocity,dt)
            output3 = int(output3)
            # print(f'output {output3}')
            a = self.mc.torquecontrol(0x01, output3)
            b = self.mc.torquecontrol(0x02, -output3)
            if a is None or b is None:
                pass
            else:
                # motorrpm = abs(a[2]) + abs(b[2])
                # motorrpm = -a[2]
                motor_speed = -(a[2]+(-b[2])) / 2 * 2 * np.pi / 60# rad/s
                # d_pos = motor_speed * 0.06152/2 * dt
                # position += d_pos
            print('speed:', motor_speed)
            desire_pitch = self.velocity_pid.update(desire_vel, motor_speed, dt)
            print('desire_pitch:', desire_pitch)
            # print(f'motor speed {motorrpm}')
            desire_pitch += middle_ang
            end = time.time()
            dt = start - end
            # print('dt', dt)
        # finally:
        #     self.disableALLmotor()

    def startController(self):
        self.prev_pitch = 0
        self.isRunning = True
        self.lockleg()
        self.pid_thread = threading.Thread(target=self.controller)
        self.pid_thread.start()

def main():
    robot_motor = robotcontrol(
        'motor01', 'motor02', 'motor11', 'motor12', 'wheel1', 'wheel2')
    command_dict = {
        "d": robot_motor.disableALLmotor,
        "start": robot_motor.startController,
        "get": robot_motor.getControllerPIDParam,
        "clear": robot_motor.mc.cleanerror,
    }

    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "set":
                set_cmd = input("choose to set Angular or Balance PID :")
                if set_cmd == "a":
                    angular_kp = input("Kp: ")
                    angular_ki = input("Ki: ")
                    robot_motor.setAngularPI(float(angular_kp), float(angular_ki))
                elif set_cmd == "b":
                    balance_kp = input("Kp: ")
                    balance_kd = input("Kd: ")
                    robot_motor.setBalancePD(float(balance_kp), float(balance_kd))
                elif set_cmd == "v":
                    vel_kp = input("Kp: ")
                    vel_ki = input("Ki: ")
                    robot_motor.setVelocityPID(float(vel_kp), float(vel_ki))
            elif cmd == "exit":
                robot_motor.disableALLmotor()
                robot_motor.closeSystem()
                break

        except Exception as e:
            traceback.print_exc()
            break


if __name__ == '__main__':

    main()

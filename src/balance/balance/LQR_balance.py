#V27 use the new LQR controller file cause the old one have the issue about negative theta error to balance
#also change the part where the controller work at, I put the control iterate inside the IMU callback rather than main()
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from sensor_msgs.msg import JointState, Imu 
from LQR import InvertedPendulumLQR
from module.DXL_motor_control import DXL_Conmunication
from module.foc_motor_serial import MotorControl

class BiwheelController:
    def __init__(self):
        #state
        self.section = 0
        self.angle_pos = [0,0]


        self.X = 0.0
        self.X_dot = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0

        self.sensor_callback_fq = 60

        self.delta_t = 1/60
        self.theta_list = []
        self.force_list = []
        self.vel_list = []

        self.linear_vel = 0.0
        self.vel_last = 0.0
        
        self.max_vel = 35.00000000 
        self.force = 0.0         
        self.force_data = []   

        self.wheel_torque = 0.0
        self.wheel_vel = 0.0
        self.Tire_radius = 0.065
        
        self.position = [0,0,0]
        self.acceleration = 0.0
        self.time = 0.0
        self.time_last = 0.0
        self.tag = 0
        self.callback_count = 0

        self.hip_con_up = math.radians(100.0)
        self.joint_state = JointState()
        self.joint_state.name = ["hip_joint_left", "hip_joint_right",
                                 "kee_joint_left", "knee_joint_right",
                                 "wheel_joint_left", "wheel_joint_right"]
        self.n = 0
        # Initialize robot state
        hip_angle_init = math.radians(75)       #default:75
        knee_angle_init = math.radians(110)     #default 110
            #In the COM computation, Hip_angle = 90 degree - hip_angle_init
                                #    Knee_angle = hip_angle_init + knee_angle_init
                                #  more detail please see the document

        self.start_flag = 0
        self.joint_state.effort = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        self.joint_state.velocity = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        self.joint_state.position = [hip_angle_init, -hip_angle_init,
                                    -knee_angle_init, knee_angle_init, 
                                    0.0 ,  0.0] 
        self.LQR = InvertedPendulumLQR(math.degrees(hip_angle_init),math.degrees(knee_angle_init))

    # def joint_states_callback(self, msg): #joint state callback freaquency : 60Hz
    #     self.Joint_rawdate_transform(msg)
        

    def imu_callback(self, msg): #imu callback frequency : 60Hz
        self.IMU_rawdata_transform(msg)
        self.get_u()
        self.send_joint_command_ros2()

        # print('x = [',self.X,'\n     ',self.X_dot,'\n     ',math.degrees(self.theta),'\n     ',self.theta_dot,']\n' ,'u =',self.force[0,0],'N\n V =',self.wheel_vel,'\n')

    def IMU_rawdata_transform(self,msg):
        self.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)
        print(self.time)
        yaw_radians = math.asin(2*(msg.orientation.w * msg.orientation.y - msg.orientation.x * msg.orientation.z)) + math.radians(12) #17.5
        self.theta_dot = (yaw_radians - self.theta)*60
        self.theta = yaw_radians
    # def Joint_rawdate_transform(self,msg):
    #     self.angle_pos[0] = msg.position[4]
    #     if (self.angle_pos[0] - self.angle_pos[1]) < -10:
    #         self.section += 1
    #     elif (self.angle_pos[0] - self.angle_pos[1]) > 10:
    #         self.section -= 1
    #     X_last = self.X
    #     self.X = self.Tire_radius * (2 * np.pi * self.section + self.angle_pos[0])
    #     self.angle_pos[1] = self.angle_pos[0]
    #     #X_dot part
    #     X_dot_knew = (self.X -X_last)*self.sensor_callback_fq  
    #     if X_dot_knew == 0.0:
    #         return
    #     self.X_dot = X_dot_knew
    #     self.X_dot = (self.X -X_last)*self.sensor_callback_fq  

    # def landing(self):
    #     land_flag = True
    #     while land_flag:
    #         self.send_joint_command_ros2()
    #         self.n +=1
    #         if self.n == 5:
    #             land_flag = False


    def Hz_count(self):
        print(self.n)
        self.n += 1 
        if self.time != self.tag :
            self.n = 0 
            self.tag = self.time

    def attitude_upgrade(self,knee,hip):
        self.joint_state.position = [hip, -hip,
                                    -knee, knee, 
                                    0.0 ,  0.0]

    def get_u(self):
        x = np.array([[self.X], [self.X_dot], [self.theta], [self.theta_dot]])
        self.force = -self.LQR.lqr_control(x) * 0.4
        print('degree :',math.degrees(self.theta),'U :',self.force[0,0])
        self.theta_list.append(math.degrees(self.theta))
        self.force_list.append(self.force)
        
        # plt.clf()
        # plt.scatter(range(len(self.theta_list)), self.theta_list)
        # plt.pause(0.00001)
        # plt.show(block = False)
        if np.isnan(self.force):
             #Sometimes LQR controller will give "nan" data, and this will trigger a bug. filter it won't cause the problem
            return
        self.wheel_vel = (self.force - 0.48*self.acceleration)/0.85
        # self.wheel_vel =60 * self.force/0.48
        
        self.wheel_vel = self.wheel_vel[0,0]/(0.0653*np.pi)       #2 * self.wheel_vel[0,0] / 0.0653 
        if abs(self.wheel_vel) >= self.max_vel:
            # self.wheel_vel = self.max_vel if self.wheel_vel > 0 else -self.max_vel
            self.wheel_vel = self.vel_last
        self.joint_state.velocity = [0.0, 0.0, 0.0,  0.0, self.wheel_vel,-self.wheel_vel]
        self.vel_list.append(self.wheel_vel)
        if len(self.theta_list) == 1000 :
                plt.scatter(range(len(self.theta_list)), self.theta_list , color = 'red')
                plt.scatter(range(len(self.force_list)), self.force_list , color = 'blue')
                plt.scatter(range(len(self.vel_list)), self.vel_list , color = 'green')
                plt.grid()
                plt.pause(0.00001)
                plt.show()
        self.vel_last = self.wheel_vel
    def send_joint_command_ros2(self):
        #position constraint
        if self.joint_state.position[0] > self.hip_con_up:
            self.joint_state.position[0] = self.hip_con_up
            self.joint_state.position[1] = -self.hip_con_up
        elif self.joint_state.position[0] < np.pi / 6:
            self.joint_state.position[0] = np.pi / 6
            self.joint_state.position[1] = -np.pi / 6

        if self.joint_state.position[3] < np.pi / 2:
            self.joint_state.position[2] = -np.pi / 2
            self.joint_state.position[3] = np.pi / 2
        elif self.joint_state.position[3] > 0.75 * np.pi:
            self.joint_state.position[2] = -0.75 * np.pi
            self.joint_state.position[3] = 0.75 * np.pi

        self.PV_pub.publish(self.joint_state)
def main():
    rclpy.init()
    biwheel_controller = BiwheelController()
    Biwheel_node = rclpy.create_node('Biwheel_node')
    biwheel_controller.PV_pub = Biwheel_node.create_publisher(JointState, 'joint_command', 1)
    # biwheel_controller.PV_sub = Biwheel_node.create_subscription(JointState, 'joint_states',
    #                                                              biwheel_controller.joint_states_callback, 1)
    biwheel_controller.imu_sub = Biwheel_node.create_subscription(Imu, 'imu/data_raw',
                                                                 biwheel_controller.imu_callback, 1)
    biwheel_controller.section = 0
    thread = threading.Thread(target=rclpy.spin, args=(Biwheel_node,), daemon=True)
    thread.start()
    rate = Biwheel_node.create_rate(100)
    # theta = [35.31 - 180 , -30 , -132.0] 
    # biwheel_controller.attitude_upgrade(math.radians(140),math.radians(65))
    

    try:
        # biwheel_controller.landing()
        print('start')
        while rclpy.ok():       #120Hz
            # biwheel_controller.send_joint_command_ros2()
            # print(math.degrees(biwheel_controller.theta))
            rate.sleep()
    except KeyboardInterrupt:
        pass

    Biwheel_node.destroy_node()
    rclpy.shutdown() 
    # plt.show()
    thread.join()

if __name__ == '__main__':
    main()


'''

Nandagopan.K

'''

'''
Working with LOW R and Q. 

SUB 1

Q1 = [[  1   0   0   0]
    [  0   1   0   0]
    [  0   0 100   0]
    [  0   0   0 100]]

also working with 

Q1 = [[ 1  0  0  0]
    [ 0  1  0  0]
    [ 0  0 10  0]
    [ 0  0  0 10]]

R = 1 * np.eye(1)

SUB 2

Q = 1 * np.eye(2)
R = 1 * np.eye(1)

'''

import rclpy
from rclpy.node import Node
# from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
import numpy as np
from example_interfaces.msg import Float64, Bool
from control.matlab import dlqr
from tf2_msgs.msg import TFMessage
from statespace import Ad, Bd, Ad2, Bd2
import time, copy

class LQRController(Node):
    def __init__(self):
        super().__init__('teeterbot_controller')

        self.imu = '/teeterbot/imu'

        self.reset_simulation_service = '/reset_world'
        self.left_torque_topic = '/left_torque_cmd'
        self.right_torque_topic = '/right_torque_cmd'

        self.create_subscription(Imu ,self.imu,self.imu_callback,100)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)
        self.pub_left = self.create_publisher(Float64, self.left_torque_topic, 100)
        self.pub_right = self.create_publisher(Float64, self.right_torque_topic, 100)
        self.timer = self.create_timer(0.01, self.control_loop)

        self.x = 3
        self.x_ = 0.0
        self.x_diff = 0.0
        self.pitch = 0.
        self.pitch_ = 0.0
        self.pitch_diff = 0.0
        self.yaw = 0.0
        self.yaw_ = 0.0
        self.yaw_diff = 0.0

        self.Ad = Ad
        self.Bd = Bd
        self.Ad2 = Ad2
        self.Bd2 = Bd2

        self.Q1 = np.array([[10, 0, 0, 0], [0, 10, 0, 0], [0, 0, 100, 0], [0, 0, 0, 100]])

        self.R1 = 1 * np.eye(1)

        self.Q2 = np.array([[100, 0],[0, 10]])
        self.R2 = 1 * np.eye(1)
        self.state1_ = np.array([[0],[0],[0],[0]])

        # self.Q1 = 5 * np.eye(4)
        # self.R1 = 1 * np.eye(1)
        # self.Q2 = 5 * np.eye(2)
        # self.R2 = 1 * np.eye(1)

        self.isFallen = False

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw    
    

    def tf_callback(self, data):
    # Iterate through the received TF messages
        for transform in data.transforms:
            if transform.child_frame_id == 'base_footprint' and transform.header.frame_id == 'world':
                self.x = transform.transform.translation.x
                # print(f"Translation from 'world' to 'base_footprint': {translation_x}")

    def imu_callback(self, msg):
            orientation = msg.orientation
        # self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

            quaternion = (
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )

            # self.x = msg.pose[teeterbot_index].position.x
            euler = self.euler_from_quaternion(orientation)
            self.pitch = euler[1]
            self.yaw = euler[2]



    def reset_simulation(self):
        reset_simulation_client = self.create_client(Empty, self.reset_simulation_service)
        while not reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = Empty.Request()
        future = reset_simulation_client.call_async(request)




    def control_loop(self):
        # print(np.rad2deg(self.pitch))
        if np.rad2deg(self.pitch) > 60 or np.rad2deg(self.pitch) < -60:
            # If the robot has fallen, stop the wheels
            left_torque = Float64(data=0.0)
            right_torque = Float64(data=0.0)
            self.pub_left.publish(left_torque)
            self.pub_right.publish(right_torque)
            self.reset_simulation()

        else:

            # state1 = np.array([[3],[0],[0],[0]])
            
            dt = 0.01
            self.x_diff = (self.x - self.x_) / dt
            self.pitch_diff = (self.pitch - self.pitch_) / dt
            self.yaw_diff = (self.yaw - self.yaw_) / dt

            self.state1 = np.array([[self.x],[self.x_diff],[self.pitch],[self.pitch_diff]])
            # print(self.state1_, self.state1)
            print(self.x)
            state2 = np.array([[self.yaw], [self.yaw_diff]])

            K1, S, e = dlqr(self.Ad, self.Bd, self.Q1, self.R1)
            K2, S, e = dlqr(self.Ad2, self.Bd2, self.Q2, self.R2)


            # print('K1: ', K1)
            # print('K2: ', K2)

            tau_t = -K1 @ self.state1  
            tau_t = tau_t[0][0]

            tau_si = -K2 @ state2
            tau_si = tau_si[0][0]



            left = float(tau_t)
            right = float(tau_si)
            l = Float64()
            l.data = left
            r = Float64()
            r.data = right


            self.pub_left.publish(l)
            self.pub_right.publish(r)


            print(self.x)

            self.x_ = self.x
            self.pitch_ = self.pitch
            self.yaw_ = self.yaw
            self.state1_ = self.state1

def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

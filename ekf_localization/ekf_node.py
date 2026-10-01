# u = robot motion commands from /odom_noisy [u = v, w]
# z = actual noisy sensor measurements (x from /odom_noisy, y from /odom_noisy, theta from /imu_noisy)

# A = not used in this EKF because our motion model is nonlinear
# B = not used in this EKF because our motion model is nonlinear
# C = not used in this EKF because we use the measurement model h and its Jacobian H

# g = predicts the robot's new state [x, y, theta] using the previous EKF state and measured v, w
# G = Jacobian of g (Used to propagate the EKF's x, y, theta uncertainty during prediction)

# h = predicts what the sensors should measure from predicted state/predicted_mean [predicted x, predicted y, predicted theta]
# H = Jacobian of h (Identity matrix because our sensors directly measure x, y, theta)

# R = motion/process noise covariance (How uncertain we are about the x, y, theta prediction from the motion model)
# Q = sensor/measurement noise covariance (How uncertain we are about x_odom, y_odom, theta_imu)

# Actual (u and z), Prediction (g and h), Jacobian (G and H)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt

class ekf_node(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # initial motion commands and timestep
        self.v = 0
        self.w = 0
        self.dt = 1/20

        # define uncertainty
        x_uncertainty = 0.1
        y_uncertainty = 0.1
        theta_uncertainty = np.deg2rad(4)

        # initialize motion and sensor uncertainty
        self.R = np.diag([0.01**2, 0.01**2, 0.005**2])
        self.Q = np.diag([0.02**2, 0.02**2, 0.01**2])

        # intialize starting state and its uncertainty
        self.previous_mean = None
        self.previous_uncertainty = np.array([[x_uncertainty**2, 0, 0],
                                 [0, y_uncertainty**2, 0],
                                 [0, 0, theta_uncertainty**2]])

        # subscribe to /odom_noisy and /imu_noisy and set timer (timestep)
        self.odom_subscription = self.create_subscription(Odometry,'/odom_noisy', self.odom_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/imu_noisy', self.imu_callback, 10)
        self.timer = self.create_timer(self.dt, self.ekf_step)

        # publish to /ekf/pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/ekf/pose', 10)

    # calculation when code receives new odometry message
    def odom_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y

        if self.previous_mean is None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # quaternion
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            # theta calculation - quaternion to yaw conversion
            siny_cosp = 2 * (qw*qz + qx*qy)
            cosy_cosp = 1 - 2 * (qy**2 + qz**2)
            theta_odom = np.arctan2(siny_cosp, cosy_cosp)

            self.previous_mean = np.array([x, y, theta_odom])

    # calculation when code receives new imu message
    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
        self.theta_imu = np.arctan2(siny_cosp, cosy_cosp)

    # Non-linear Motion Model g (predicted state) and its Jacobian G (This function predicts the new state [x, y, θ] using previous state and v, w)
    def motion_model(self, state, u):
        x = state[0]
        y = state[1]
        theta = state[2]
        v = u[0]
        w = u[1]

        if abs(w) < 1e-6:
            g = np.array([
                x + v * self.dt * np.cos(theta),
                y + v * self.dt * np.sin(theta),
                theta
            ])

            G = np.array([
                [1, 0, -v * self.dt * np.sin(theta)],
                [0, 1,  v * self.dt * np.cos(theta)],
                [0, 0, 1]
            ])
            return g, G

        else:
            g = np.array([
                x - (v/w) * np.sin(theta) + (v/w) * np.sin(theta + w*self.dt),
                y + (v/w) * np.cos(theta) - (v/w) * np.cos(theta + w*self.dt),
                theta + w*self.dt
            ])

            G = np.array([
                [1, 0, (v/w) * (-np.cos(theta) + np.cos(theta + w*self.dt))],
                [0, 1, (v/w) * (-np.sin(theta) + np.sin(theta + w*self.dt))],
                [0, 0, 1]
            ])

        return g, G

    # Measurement Model h and its Jacobian H (This function predicts what the sensors should measure given the predicted EKF state.)
    def measurement_model(self, state):
        h = state
        H = np.identity(3)
        return h, H

    def prediction(self, previous_mean, previous_uncertainty, u):
        g, G = self.motion_model(previous_mean, u)
        predicted_mean = g
        predicted_uncertainty = G @ previous_uncertainty @ G.T + self.R  # same thing as predicted_covariance_matrix
        return predicted_mean, predicted_uncertainty

    def correction(self, predicted_mean, predicted_uncertainty, z):
        h, H = self.measurement_model(predicted_mean)
        kalman_gain = predicted_uncertainty @ H.T @ np.linalg.inv(H @ predicted_uncertainty @ H.T + self.Q)
        mean = predicted_mean + kalman_gain @ (z - h)
        uncertainty = (np.identity(3) - kalman_gain @ H) @ predicted_uncertainty
        return mean, uncertainty

    def ekf_step(self):
        if self.previous_mean is None:
            return
        
        u = np.array([self.v, self.w])
        predicted_mean, predicted_uncertainty = self.prediction(self.previous_mean, self.previous_uncertainty, u)

        if self.x_odom == None or self.y_odom == None or self.theta_imu == None:
            return

        z = np.array([self.x_odom, self.y_odom, self.theta_imu])  # initialize actual sensor measurements (noisy)
        mean, uncertainty = self.correction(predicted_mean, predicted_uncertainty, z)
        self.previous_mean = mean
        self.previous_uncertainty = uncertainty

        # create message (pose_msg) to publish
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom' # Why odom? Because the EKF pose is being estimated in the same coordinate frame as the odometry we're using.

        # fill message with position values
        pose_msg.pose.pose.position.x = mean[0]
        pose_msg.pose.pose.position.y = mean[1]
        pose_msg.pose.pose.position.z = 0.0
        theta = mean[2]

        # convert above theta to quaternion
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = np.cos(theta / 2.0)

        # fill message with uncertainty values
        pose_msg.pose.covariance[0] = uncertainty[0, 0]    # x variance
        pose_msg.pose.covariance[1] = uncertainty[0, 1]    # x-y covariance
        pose_msg.pose.covariance[5] = uncertainty[0, 2]    # x-theta covariance

        pose_msg.pose.covariance[6] = uncertainty[1, 0]    # y-x covariance
        pose_msg.pose.covariance[7] = uncertainty[1, 1]    # y variance
        pose_msg.pose.covariance[11] = uncertainty[1, 2]   # y-theta covariance

        pose_msg.pose.covariance[30] = uncertainty[2, 0]   # theta-x covariance
        pose_msg.pose.covariance[31] = uncertainty[2, 1]   # theta-y covariance
        pose_msg.pose.covariance[35] = uncertainty[2, 2]   # theta variance

        self.pose_publisher.publish(pose_msg)

'''
        x   y   z  roll pitch yaw
x         0   1   2   3    4    5
y         6   7   8   9   10   11
z        12  13  14  15   16   17
roll     18  19  20  21   22   23
pitch    24  25  26  27   28   29
yaw      30  31  32  33   34   35
'''

def main(args=None):

    rclpy.init(args=args)
    node = ekf_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
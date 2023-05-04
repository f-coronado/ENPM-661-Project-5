import rclpy
from rclpy.node import Node
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

from angle_helpers import euler_from_quaternion


class PurePursuit(Node):

    def __init__(self, plan, lookahead_distance=0.4, kp = 0.3):
        
        self.plan = plan
        self.lookahead_distance = lookahead_distance
        self.kp = kp
        self.traj = []
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.sample_time = 0.1

        self.lidar_readings = []

        self.Kp = 0.5
        self.Kd = 0.045
        self.Ki = 0.005

        self.error = 0
        self.prev_error = 0
        self.cumulative_error = 0

        self.max_v = 1.3
        self.k_brake_large = 0.4
        self.k_brake_ahead = 1.1

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        self.create_subscription(LaserScan, 'scan', self.call_Lidar, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub.publish(self.vel)

        self.out = False


    def call_Lidar(self, msg):

        self.lidar_readings = msg.ranges

    def call_position(self, msg):

        euler_angles = euler_from_quaternion( [msg.pose.pose.orientation.x,
                                               msg.pose.pose.orientation.y,
                                               msg.pose.pose.orientation.z,
                                               msg.pose.pose.orientation.w])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_angles[2]





    def pursue(self):

        xt = list(x[0] for x in self.traj)
        yt = list(x[1] for x in self.traj)

        d_arc = 0
        step = 0


        # move about look ahead distance
        while d_arc < self.lookahead_distance and step <= len(xt):
            d_arc += math.sqrt((xt[step+1] - xt[step])**2 + (yt[step+1] - yt[step])**2)
    
        marker = self.create_waypoint_marker(xt[step+1], yt[step+1], nearest_wp=True)

        # obtain radius: all coordinates are already in local frame

        L_sq = (xt[step])**2 + (yt[step])**2
        radius = L_sq / (2 * yt[step])

        self.error = 1/radius

        self.cumulative_error = self.error * self.sample_time
        self.vel.angular.z = self.kp * self.error + self.Kd * (self.error - self.prev_error)/self.sample_time + self.Ki * (self.cumulative_error)

        self.prev_error = self.error

        if len(self.lidar_readings) > 0:
            wall_ahead = np.array([*self.lidar_readings[-30:], *self.lidar_readings[:30]])
            large_dist = np.min(wall_ahead)
            ahead_dist = np.min(wall_ahead[25:35])

            # compute brake factor

            brake = 1 + np.abs(math.atan2(yt[step], xt[step]))
            brake += self.k_brake_large* 1/(large_dist)
            brake += self.k_brake_ahead* 1/(ahead_dist)
            self.vel.linear.x = self.max_v / brake

        self.pub.publish(self.vel)

        return marker


    def create_waypoint_marker(self, wp_x, wp_y, nearest_wp=False):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now()
        marker.id = 0
        marker.type = 2
        marker.action = 0
        marker.pose.position.x = wp_x
        marker.pose.position.y = wp_y
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0

        if nearest_wp:

            marker.scale.x *= 2
            marker.scale.y *= 2
            marker.scale.z *= 2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        else:

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        return marker
    




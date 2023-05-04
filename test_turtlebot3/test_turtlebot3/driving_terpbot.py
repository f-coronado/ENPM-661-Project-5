import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
import numpy as np
import math as math

from angle_helpers import euler_from_quaternion


class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        scan_topic = '/scan'
        pf_topic = '/odom'



        self.width = 25
        self.height = 25
        self.resolution = 0.1
        self.footprint = 0.3
        


        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE


        self.occ_pub = self.create_publisher(OccupancyGrid, '/map', queue_size = 10)
        self.subscription= self.create_subscription(
            LaserScan,
            scan_topic,
            self.listener_callback,
            qos_profile,
        )
        self.pose_sub = self.create_subscription(Odometry, pf_topic, self.pose_callback, 1)

        self.curr_pos = [0,0]
        self.curr_orientation = [0,0,0,1]




        







    def listener_callback(self,msg):
        self.get_logger().info('I heard : Range[0] "%f" Ranges[50]: "%f"' %(msg.ranges[0] ,msg.ranges[50]))


        grid = np.ndarray((self.width, self.height), buffer=np.zeros((self.width, self.height), dtype= np.int), dtype=np.int)
        grid.fill(int(-1))

        angle_min = msg.angle_min
        angle_max = msg.angle_max

        dist = list(msg.ranges)
        angles = np.linspace(angle_min, angle_max, num = len(dist))

    
    def pose_callback(self, msg):

         if self.target is not None:
            x, y = self.target

            position_x = 5 - msg.transform.translation.x
            position_y = 5 - msg.transform.translation.y
            self.get_logger().info(f' position{(position_x, position_y)}')
            # calculate distance to target
            distance = np.sqrt((x - position_x) ** 2
                                  + (y - position_y) ** 2)
            self.get_logger().info(str(distance))
            if distance < 0.02:
                # reached target
                self.last_target = self.target
                self.target = None

            _, _, theta = euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y,
                                                                    msg.transform.rotation.z, msg.transform.rotation.w])
            theta += math.pi
            steering_angle = math.atan2(y - position_y, x - position_x)
            rad = steering_angle - theta
            if rad > math.pi:
                rad -= 2. * math.pi
            elif rad < -math.pi:
                rad += 2. * math.pi

            move = Twist()
            move.linear.x = 0.03  # might want to change constant
            move.angular.z = math.sin(steering_angle - theta)

            self.get_logger().info(f'distance {distance} s_angle{steering_angle}, theta{theta} rad {rad}')
            self.get_logger().info(f' x{move.linear.x} ,z{move.angular.z}')

            self.twist.publish(move)
            self.search_counter = 0

        


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
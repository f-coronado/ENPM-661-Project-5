import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import time
from angle_helpers import euler_from_quaternion

class TurtlebotController(Node):

    def __init__(self):
        super().__init__("turtlebot_controller")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription_ = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.current_position = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = None
        # self.subscription_  # prevent unused variable warning

    def read_positions_from_file(self, filename):
        with open(filename) as f:
            positions = [line.rstrip() for line in f]
        return positions



    def odom_callback(self, msg : Odometry):
                    # Compute the angle between the TurtleBot's current position and the target position
            
            positions = self.read_positions_from_file("/home/landis/turtlebot3_ws/src/test_turtlebot3/test_turtlebot3/output_RRT_results.txt")
            
            

            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_position = (self.current_x, self.current_y, 0.0)
          

            k = 0.5

            for position in positions:
                x, y = position.split(" ")
                x = float(x)
                y = float(y)


                rate = self.create_rate(10) # loop at 10 Hz
                while self.current_position is None or self.current_heading is None:
                    rate.sleep()



                    angle_to_target = math.atan2(y - self.current_y, x - self.current_x)

                    # Compute the error between the target angle and the TurtleBot's current heading
                    current_heading = self.current_heading

                    x = msg.pose.pose.position.x
                    y = msg.pose.pose.position.y

                    quaternion = (
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w,
                    )

                    roll, pitch, yaw = euler_from_quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                    current_heading = yaw

                    # Set the current position and heading
                    self.current_position = (x, y, 0.0)
                    self.current_heading = current_heading

                    # Print the current position and heading for debugging
                    print(f"Current position: {self.current_position}, Current heading: {self.current_heading}")

                    if self.current_position is not None:
                        self.current_x, self.current_y, _ = self.current_position
                        error = angle_to_target - current_heading

                        # Compute the angular velocity proportional to the error
                        angular_velocity = k * error

                        twist = Twist()
                        dx = x - self.current_x
                        dy = y - self.current_y
                        norm = math.sqrt(dx**2 + dy**2)
                        twist.linear = Vector3(dx/norm, dy/norm, 0.0)
                      
                        twist.angular = Vector3(0.0, 0.0, angular_velocity)
                        self.publisher_.publish(twist)
                        time.sleep(1.0)

      


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

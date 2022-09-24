from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
import math
from math import isinf

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        # Create a timer that fires ten times per second
        self.create_timer(0.1, self.run_loop)
        # Create subscribers to LaserScan and Bump
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Bump, 'bump', self.read_bump, qos_profile= qos_profile_sensor_data)
        # Create publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Initialize Twist
        self.msg = Twist()
        # Initialize x position
        self.position_x = 0
        # Initialize y position
        self.position_y = 0
        # Initialize angle
        self.angle = 0
        # Initialize distance from obstacle
        self.distance = None
        # 
        self.points = []
        # set parameters
        self.base_angular_speed = .2

    def run_loop(self):
        """
        """
        if self.distance != None and isinf(self.distance) is False:
                point = self.point_position(10 - self.distance) # closer points should have more magnitude
                self.points.append(point)


                # sum x and y values to get net force in x and y directions
                x_sum = 100 # weight it so it wants to go forward more
                y_sum = 0
                for point in self.points:
                    x_sum += point.x
                    y_sum += point.y

                # find the resulting angle/magnitude of the summed forces
                self.angle, magnitude = self.calculate_angular_velocity(x_sum, y_sum)

                # if the force is pushing forwards (<90, >270), the robot doesn't need to change its behavior because there's no obvious obstacle in its path yet;
                # if the force is pushing backwards (>90, <270), however, it means there's something in front of the robot and it needs to turn
                if self.angle > 90 or self.angle < 270:
                    angle_proportion = math.sin(math.radians(self.angle))
                    magnitude_proportion = (magnitude ** (.3)) #0.125
                    self.msg.angular.z = self.base_angular_speed * angle_proportion * magnitude_proportion
                    self.vel_pub.publish(self.msg)

        else:
            self.msg.linear.x = 0.5
            self.msg.angular.z = 0.0
            self.vel_pub.publish(self.msg)


    def process_scan(self, msg:LaserScan):
        """
        """
        for self.angle in range(360):
            self.distance = msg.ranges[self.angle]

    def point_position(self, distance):
        """
        """
        self.angle = math.radians(self.angle + 180)      #radians
        x_position = math.cos(self.angle) * distance
        y_position = math.sin(self.angle) * distance
        return Vector3(x = x_position, y = y_position)

    def calculate_angular_velocity(self, x, y):
        """
        """
        if y == 0:
            y = 0.00000001 # so we don't get division by zero complaints
        self.angle = math.degrees(math.atan(x/y))
        magnitude = math.sqrt(x**2 + y**2)
        return (self.angle, magnitude)

    def read_bump(self, msg:Bump):
        """
        """
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.vel_pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)       # Initialize communication with ROS
    node = ObstacleAvoider()    # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    rclpy.shutdown()            # Cleanup

if __name__ == '__main__':
    main()
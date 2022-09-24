import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 

class DriveSquare(Node):

    def __init__(self):
        super().__init__('drive_square_node')
        # Create a timer that fires ten times per second
        self.create_timer(0.1, self.run_loop)
        # Create publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Length of side of square
        self.leg_length = 1.0       # meters
        # Time for the side
        self.leg_time = 3.0         # seconds
        # Time for the turn
        self.turn_time = 2.0        # seconds

    def run_loop(self):
        msg = Twist()
        self.straight(msg)
        self.turn(msg)

    def straight(self, msg:Twist):
        """
        Based on the time and length of the side of the square, publish the linear velocity
        """
        # Get the start time of the method
        start_time = self.get_clock().now()
        # While during the time to transverse a leg, publish the linear velocity
        while (self.get_clock().now() - start_time) < rclpy.time.Duration(seconds = self.leg_time):
            # Set the linear and angular velocity (m/s)
            msg.linear.x = self.leg_length / self.leg_time
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)

    def turn(self, msg:Twist):
        """
        Based on the time, turn the Neato 90 degrees
        """
        # Get the start time of the method
        start_time = self.get_clock().now()
        # While during the time to turn 90 degreees, publish the angular velocity
        while (self.get_clock().now() - start_time) < rclpy.time.Duration(seconds = self.turn_time):
            # Set the linear and angular velocity (m/s)
            msg.angular.z = (1.5708) / self.turn_time
            msg.linear.x = 0.0
            self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)       # Initialize communication with ROS
    node = DriveSquare()        # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    rclpy.shutdown()            # Cleanup


if __name__ == '__main__':
    main()

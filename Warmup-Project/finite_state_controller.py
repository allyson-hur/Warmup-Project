import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np

# Distance and angle proportion values
KP_DIST = 1
KP_ANGLE = 1

class FiniteStateController(Node):

    def __init__(self):
        super().__init__('finite_state_node')
        # Create a timer that fires ten times per second
        self.create_timer(0.1, self.run_loop)
        # Create publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create subscriber
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        # Length of side of square
        self.leg_length = 1.0       # meters
        # Time for the side
        self.leg_time = 3.0         # seconds
        # Time for the turn
        self.turn_time = 2.0        # seconds
        # Initialize optimal distance
        self.target_distance = 1.2
        # Initialize list of LiDAR points
        self.det_poses = []

    def run_loop(self):
        """
        Finite-State controller. If there are no LiDAR points detected, the Neato
        will run the drive_square operation. If there are points detected, the Neato
        will enter the person_follower operation.
        """
        msg = Twist()
        if not self.det_poses:
            self.straight(msg)
            self.turn(msg)
            return
        # Find mean of the LiDAR points
        lidar_poses = np.mean(np.array(self.det_poses), axis=0)
        angle = lidar_poses[1]
        dist = lidar_poses[0]
        #print(f"{angle}  {dist}")
        # Publish the linear and angular velocities based on the measured
        # distance and angle. Both are weighted by a proportional constant
        self.vel_pub.publish(Twist(linear=Vector3(x=(dist-self.target_distance)*KP_DIST), angular=Vector3(z=angle*KP_ANGLE)))

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

    def process_scan(self, msg:LaserScan):
        """
        Processes the LaserScan from the Neato.
        """
        angle = 0
        self.det_poses = []
        for range in msg.ranges:
            # Only access points that are 3 meters away
            if range != 0.0 and range < 3:
                self.det_poses.append([range, angle])
            angle += msg.angle_increment
            # Make angles greater than by pi negative
            if angle > math.pi:
                angle = -math.pi

def main(args=None):
    rclpy.init(args=args)               # Initialize communication with ROS
    node = FiniteStateController()      # Create our Node
    rclpy.spin(node)                    # Run the Node until ready to shutdown 
    rclpy.shutdown()                    # Cleanup


if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np

KP_DIST = 1
KP_ANGLE = 1

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Create publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create subscriber
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        # Initialize Twist
        self.vel = Twist()
        # Initialize linear velocity
        self.vel.linear.x = 0.0
        # Initialize angular velocity
        self.vel.angular.z = 0.0
        # Initialize optimal distance
        self.target_distance = 1.2
        # Initialize list of LiDAR points
        self.det_poses = []

    def run_loop(self):
        """
        Main run_loop of the Person Follower Node. Finds the 
        mean of the detected LiDAR points, and given that angle and 
        distance travels to the object.
        """
        # Stop Neato if no LiDAR points are within range
        if not self.det_poses:
            self.publisher.publish(Twist())
            return
        # Find mean of the LiDAR points
        lidar_poses = np.mean(np.array(self.det_poses), axis=0)
        angle = lidar_poses[1]
        dist = lidar_poses[0]
        print(f"{angle}  {dist}")
        # Publish the linear and angular velocities based on the measured
        # distance and angle. Both are weighted by a proportional constant
        self.publisher.publish(Twist(linear=Vector3(x=(dist-self.target_distance)*KP_DIST), angular=Vector3(z=angle*KP_ANGLE)))


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
    rclpy.init(args=args)       # Initialize communication with ROS
    node = PersonFollower()     # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    rclpy.shutdown()            # Cleanup


if __name__ == '__main__':
    main()





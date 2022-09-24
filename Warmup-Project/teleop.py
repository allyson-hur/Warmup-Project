import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Create publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Initalize settings for keys
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        # Initialize Twist
        self.vel = Twist()
        # Initialize linear velocity
        self.vel.linear.x = 0.0
        # Initialize angular velocity
        self.vel.angular.z = 0.0


    def getKey(self):
        """
        Returns the pressed key while in the terminal window
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key

    
    def run_loop(self):
        """
        Gets the pressed key from the terminal window and publishes the
        set movement
        """
        # 'CTRL-C' will kill the ROS2 command and the run_loop
        while self.key != '\x03':
            self.key = self.getKey()
            # Move Neato forward when 'w' key is pressed
            if self.key == 'w':
                self.vel.linear.x = 0.5
                self.publisher.publish(self.vel)
            # Turn Neato left when 'a' key is pressed
            if self.key == 'a':
                self.vel.angular.z = 1.0
                self.publisher.publish(self.vel)
            # Reverse Neato when 's' key is pressed
            if self.key == 's':
                self.vel.linear.x = -0.5
                self.publisher.publish(self.vel)
            # Turn Neato right when 'd' key is pressed
            if self.key == 'd':
                self.vel.angular.z = -1.0
                self.publisher.publish(self.vel)
            # Stop Neato when the spacebar is pressed
            if self.key == ' ':
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.0
                self.publisher.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)       # Initialize communication with ROS
    node = Teleop()             # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    rclpy.shutdown()            # Cleanup


if __name__ == '__main__':
    main()





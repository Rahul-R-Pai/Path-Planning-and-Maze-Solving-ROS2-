import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import sys
import math


class robot_go_to_goal(Node):

    def __init__(self):
        super().__init__('goal_movement_node')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry,'/odom',self.pose_callback,10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal_function)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0
        self.angle_offset = 0

    def pose_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.robot_pose.z = yaw
        

    def go_to_goal_function(self):

        self.goal_pose.x = float(sys.argv[1])
        self.goal_pose.y = float(sys.argv[2])
        self.angle_offset= float(sys.argv[3])

        self.distance_to_goal = pow( pow((self.goal_pose.x-self.robot_pose.x),2) + pow((self.goal_pose.y-self.robot_pose.y),2), 0.5)
        self.angle_to_goal = math.atan2((self.goal_pose.y-self.robot_pose.y),(self.goal_pose.x-self.robot_pose.x)) + self.angle_offset
        self.angle_to_goal = self.angle_to_goal - self.robot_pose.z

        if abs(self.angle_to_goal) > 0.1:
            self.vel_msg.angular.z = self.angle_to_goal
            self.vel_msg.linear.x = 0.0
        else:
            self.vel_msg.linear.x = self.distance_to_goal


        msg = 'DTG: {:3f}  ATT: {:3f}'.format(self.distance_to_goal, self.angle_to_goal)
        self.get_logger().info(msg)
        self.velocity_pub.publish(self.vel_msg)

    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)

    gtg_node = robot_go_to_goal()
    rclpy.spin(gtg_node)
    gtg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
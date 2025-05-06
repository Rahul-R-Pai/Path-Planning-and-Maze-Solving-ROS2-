import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from .bot_localization import bot_localizer

class maze_solver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)

        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg=Twist()

        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.sat_view = np.zeros((100,100))

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
        cv2.imshow("sat_view", self.sat_view)
        cv2.waitKey(1)

    def maze_solving(self): 
        ## Write all required functionalities below
        ## this function is called every 0.2 secs
        frame_disp = self.sat_view.copy()
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)

        self.vel_msg.linear.x = 0.01 # for very slow movement
        self.vel_msg.angular.z = 0.0

        self.velocity_publisher.publish(self.vel_msg)

def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
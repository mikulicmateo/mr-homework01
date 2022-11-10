import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Point, Vector3
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry

goal_position = Point()
current_position = Point()
current_yaw = 0.0
Kp = 0.2
linear_speed_const = 0.05 

class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')
        self.create_subscription(Odometry, "/odom", self.current_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_changed_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)


    def goal_changed_callback(self, pose_stamped):
        global goal_position
        goal_position = pose_stamped.pose.position


    def current_position_callback(self, odom):
        global current_position, current_yaw 
        current_position = odom.pose.pose.position
        
        orientation_quat = odom.pose.pose.orientation
        r =  Rotation.from_quat([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        current_yaw = r.as_euler('xyz', degrees=False)[2]
        
        if goal_position.x != 0.0 and goal_position.y != 0.0:
            self.move_to_goal()


    def move_to_goal(self):
        
        delta_yaw = self.calculate_delta_yaw()
        
        angle = delta_yaw - current_yaw
        goal_delta_x = goal_position.x - current_position.x
        goal_delta_y = goal_position.y - current_position.y

        if angle > math.pi:
            angle -= 2*math.pi
        if angle <= -math.pi:
            angle += 2*math.pi

        command = Twist()
        if not 0 <= abs(angle) < 0.01:            
            command.angular = Vector3(x=0.0, y=0.0, z=Kp*angle)
        else:
            command.angular = Vector3(x=0.0, y=0.0, z=0.0)

        if 0 <= abs(goal_delta_x) < 0.01 and 0 <= abs(goal_delta_y) < 0.01:
            command.linear = Vector3(x=0.0, y=0.0, z=0.0)
            command.angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:
            if abs(angle) > math.pi/6.:
                command.linear = Vector3(x=0.0, y=0.0, z=0.0)
            else:
                command.linear = self.adjust_linear_speed(goal_delta_x, goal_delta_y)
        
        self.publisher_.publish(command)
    

    def adjust_linear_speed(self, goal_delta_x, goal_delta_y):
        if 0 <= abs(goal_delta_x) < 0.1 and 0 <= abs(goal_delta_y) < 0.1:
            return Vector3(x=linear_speed_const/2., y=0.0, z=0.0)
        else:
            return Vector3(x=linear_speed_const, y=0.0, z=0.0)


    def calculate_delta_yaw(self):
        return math.atan2(goal_position.y - current_position.y, goal_position.x - current_position.x)



def main(args = None):
    rclpy.init(args=args)

    position_controller =  PositionController()
    rclpy.spin(position_controller)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
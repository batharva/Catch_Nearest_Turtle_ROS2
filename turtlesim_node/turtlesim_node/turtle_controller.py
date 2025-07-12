import rclpy
from rclpy.node import Node
import math
from functools import partial
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interface.msg import Turtle
from my_robot_interface.msg import TurtleArray
from my_robot_interface.srv import CatchTurtle
class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.turtle_to_catch_= None
        self.current_pose :Pose= None
        self.declare_parameter("catch_closest_turtle_first",True)
        self.catch_closest_turtle_ = self.get_parameter("catch_closest_turtle_first").value
        self.get_current_position_subscriber_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback_get_current_position,
            10
        )

        self.move_towards_target_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer_ = self.create_timer(0.01, self.callback_move_towards_target_)

        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles",self.callback_alive_turtles,10)

        self.catch_turtle_client_ = self.create_client(CatchTurtle,"catch_turtle")
    def callback_alive_turtles(self,msg):
        if len(msg.turtles) >0:
            if self.catch_closest_turtle_:
                closent_turtle = None
                closent_turtle_distance = 0
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.current_pose.x
                    dist_y = turtle.y - self.current_pose.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closent_turtle == None or distance < closent_turtle_distance:
                        closent_turtle = turtle
                        closent_turtle_distance = distance
                self.turtle_to_catch_ = closent_turtle
            else:
                self.turtle_to_catch_ = msg.tuetles[0]
    def call_catch_turtle_service(self,turtle_name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
           self.get_logger().info("Waiting for catch turtle service..")

        request = CatchTurtle.Request()
        request.name = turtle_name
        
        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service,turtle_name=turtle_name))

    def callback_call_catch_turtle_service(self,future,turtle_name):
        response = future.result()
        if not response.success:
            self.get_logger().info("Turtle "+ turtle_name+ " could not be remove." )

    def callback_get_current_position(self, pose: Pose):
        self.current_pose = pose  

    def callback_move_towards_target_(self):
        if self.current_pose is None or self.turtle_to_catch_ is None:
            return

        dist_x = self.turtle_to_catch_.x - self.current_pose.x  
        dist_y = self.turtle_to_catch_.y - self.current_pose.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        cmd = Twist()

        if distance > 0.5:
            cmd.linear.x = distance * 2.0

            goal_theta = math.atan2(dist_y, dist_x)
            diff_theta = goal_theta - self.current_pose.theta

            # Normalize angle difference between -pi and pi
            diff_theta = math.atan2(math.sin(diff_theta), math.cos(diff_theta))

            cmd.angular.z = diff_theta * 6.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.move_towards_target_.publish(cmd)  

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

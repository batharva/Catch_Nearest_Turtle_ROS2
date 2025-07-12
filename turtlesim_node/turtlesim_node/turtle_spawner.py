import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
from my_robot_interface.msg import Turtle
from my_robot_interface.msg import TurtleArray
from my_robot_interface.srv import CatchTurtle 

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.declare_parameter("turtle_name_prefix","spawned_turtle")
        self.spawned_turtle_name_prefix_= self.get_parameter("turtle_name_prefix").value
        self.declare_parameter("spawn_frequency",1.0)
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.spawned_turtle_counter_ =0 
        self.alive_turtles_ = []
        self.turtle_spawner_client_ = self.create_client(Spawn,'/spawn')
        self.spawn_turtle_timer_ = self.create_timer((1.0/self.spawn_frequency_),self.spawn_new_turtle)
        self.alive_turtles_publisher_= self.create_publisher(TurtleArray,'alive_turtles',10)
        self.catch_turtle_service_ = self.create_service(CatchTurtle,'catch_turtle',self.callback_catch_turtle)
        self.kill_client_ = self.create_client(Kill,"/kill")
    def callback_catch_turtle(self, request,response):
        self.call_kill_service(request.name)
        response.success = True
        return response
    def call_kill_service(self,turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for kill service..")
        request = Kill.Request()
        request.name = turtle_name
        
        future = self.kill_client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service,turtle_name=turtle_name))
    def callback_call_kill_service(self,future,turtle_name):
        for (i,turtle) in enumerate(self.alive_turtles_):
            if turtle.name == turtle_name:
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                break
                
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def call_turtle_spawner_client(self,x,y,theta,name):
        while not self.turtle_spawner_client_.wait_for_service(1.0):
           self.get_logger().warn("Waiting for TurtleSim Server.")
        request  = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.turtle_spawner_client_.call_async(request)
        future.add_done_callback(partial(self.callback_turtle_spawner,request=request))

    def callback_turtle_spawner(self,future,request):
        response: Spawn.Response= future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: "+response.name)
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()
       

    def spawn_new_turtle(self):
        self.spawned_turtle_counter_ +=1
        name = self.spawned_turtle_name_prefix_+str(self.spawned_turtle_counter_)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0.0,2*math.pi)
        self.call_turtle_spawner_client(x,y,theta,name)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    node.call_turtle_spawner_client(9.1,2.1,3.14,'spawnedturtle')
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
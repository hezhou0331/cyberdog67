import rclpy                                # ROS 2 Python interface
from rclpy.node import Node                 # ROS 2 Node class
from protocol.msg import MotionServoCmd     # Motion cmd msg type
from std_msgs.msg import String             # String msg type

class UltrasonicMove(Node):
    def __init__(self, name):                          
        super().__init__(name)    
        self.dog_name = "dog1"                
        # Create subscriber for obstacle detection results
        self.sub = self.create_subscription(
            String, 
            "ultrasonic_obstacle", 
            self.ultrasonic_move_callback, 
            10
        )
        # Create publisher for motion cmds
        self.pub = self.create_publisher(MotionServoCmd, f'/{self.dog_name}/motion_servo_cmd', 10)

    def ultrasonic_move_callback(self, msg):
        obstacle_detected = msg.data
        motion_cmd = MotionServoCmd()

        if obstacle_detected == "True":
            self.get_logger().info("Obstacle detected! Modifying trajectory...")
            motion_cmd.motion_id = 303                 # ID 303: Specific motion for stepping
            motion_cmd.cmd_type = 1                    # Cmd type 1: Execute
            motion_cmd.value = 2                       # Mode value
            motion_cmd.vel_des = [0.0, 0.0, 0.5]       # Turn left
            motion_cmd.step_height = [0.05, 0.05]      # 5cm step height

        else:
            self.get_logger().info("Path is clear. Continuing forward...")
            motion_cmd.motion_id = 303                 # ID 303: Specific motion for stepping
            motion_cmd.cmd_type = 1                    # Cmd type 1: Execute
            motion_cmd.value = 2                       # Mode value
            motion_cmd.vel_des = [0.2, 0.0, 0.0]       # Move forward in the x-axis
            motion_cmd.step_height = [0.05, 0.05]      # 5cm step height

        self.pub.publish(motion_cmd)

def main(args=None):
    rclpy.init(args=args)                              
    node = UltrasonicMove("move_the_dog")                                                  
    rclpy.spin(node)                                   # Keep the node running
    node.destroy_node()                                
    rclpy.shutdown()

if __name__ == "__main__":
    main()
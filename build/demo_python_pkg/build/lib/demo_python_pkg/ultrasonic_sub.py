import rclpy                                                            # ROS 2 Python client library
from rclpy.node import Node                                             # ROS 2 Node class
from sensor_msgs.msg import Range                                       # ROS 2 Range msg 
from std_msgs.msg import String                                         # String msg type

class UltrasonicSubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)                                          # Initialize the parent Node class
        # Create subscription obj (Msg Type, Topic Name, Callback, Queue Size)                                                 
        self.sub = self.create_subscription(
            Range, 
            "/dog1/ultrasonic_payload", 
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(String, "ultrasonic_obstacle", 10) 
        self.obstacle_threshold = 0.5                                             # Threshold distance in m for obstacle detection      

    def listener_callback(self, msg): 
        dist = msg.range                                 
        self.get_logger().info(f"Received distance: {dist} m")              # Output log info with the received msg

        # Determine if an obstacle is detected
        obstacle_detected = dist < self.obstacle_threshold
        self.get_logger().info(f"Obstacle detected: {obstacle_detected}")

        # Publish the obstacle detection result as a String msg
        result_msg = String()
        result_msg.data = str(obstacle_detected)                        # Convert boolean to string for publishing
        self.pub.publish(result_msg)
        self.get_logger().info(f"Published obstacle detection result: {result_msg.data}")

def main(args=None):                                                    # Main entry fct for the ROS 2 node
    rclpy.init(args=args)                                               # Initialize the ROS 2 Python interface
    node = UltrasonicSubscriberNode("topic_ultrasonic_sub")                       # Create and initialize the ROS 2 node obj
    rclpy.spin(node)                                                    # Block and keep the node active to receive msgs
    node.destroy_node()                                                 # Destroy the node obj        
    rclpy.shutdown()                                                    # Shutdown the ROS 2 Python interface   

if __name__ == '__main__':                                              # Standard Python execution entry pt                                       
    main()                                                              # Call the main fct
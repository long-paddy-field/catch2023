import rclpy
from rclpy.node import Node


class launcher_catch(Node):
    def __init__(self):
        super().__init__("launcher_catch")
        self.get_logger().info("launcher_catch start")
        
    

    def __del__(self):
        self.get_logger().info("launcher_catch end")


def main():
    rclpy.init()
    node = rclpy.create_node("launcher_catch")
    node.get_logger().info("launcher_catch start")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

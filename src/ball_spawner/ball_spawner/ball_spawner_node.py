#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import random



class BallSpawner(Node):
    def __init__(self):
        super().__init__("ball_spawner")


        # Publishes the ball pose (for future use)
        self.ball_pub = self.create_publisher(PoseStamped, "ball_pose", 10)


        # Publishes a marker (visual sphere in RViz)
        self.marker_pub = self.create_publisher(Marker, "ball_marker", 10)


        # Spawn ball every 5 sec
        self.timer = self.create_timer(5.0, self.spawn_ball)


        self.get_logger().info("Ball spawner running...")


    def spawn_ball(self):
        # Random coordinates in front of G1
        x = random.uniform(0.25, 0.6)
        y = random.uniform(-0.3, 0.3)
        z = random.uniform(0.6, 0.1)


        # Publish PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = "pelvis"   # matches your G1 model
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0


        self.ball_pub.publish(pose)


        # Create RViz marker sphere
        marker = Marker()
        marker.header = pose.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.09  # 9 cm ball
        marker.scale.y = 0.09
        marker.scale.z = 0.09
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose = pose.pose


        self.marker_pub.publish(marker)


        self.get_logger().info(
            f"Spawned ball at ({x:.2f}, {y:.2f}, {z:.2f})"
        )



def main(args=None):
    rclpy.init(args=args)
    node = BallSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




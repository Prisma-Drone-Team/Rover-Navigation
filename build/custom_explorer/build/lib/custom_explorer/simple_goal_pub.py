#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class SimpleGoalPublisher(Node):
    def __init__(self):
        super().__init__('simple_goal_publisher')
        
        # Publisher su /robot1/goal_pose
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            '/robot1/goal_pose', 
            10
        )
        
        # ✅ 6 goal per coprire area 5m² (griglia 2x3)
        # Start: (20.0, -3.0)
        self.goals = [
            (20.0, -3.0),   # Goal 1: Centro (start)
            (18.5, -3.0),   # Goal 2: Sinistra
            (21.5, -3.0),   # Goal 3: Destra
            (20.0, -1.5),   # Goal 4: Nord
            (18.5, -1.5),   # Goal 5: Nord-Ovest
            (21.5, -1.5),   # Goal 6: Nord-Est
        ]
        
        self.current_goal_idx = 0
        
        # Timer: pubblica ogni 20 secondi
        self.timer = self.create_timer(30.0, self.publish_next_goal)
        
        self.get_logger().info("🎯 Simple Goal Publisher Ready")
        self.get_logger().info(f"📍 Will publish {len(self.goals)} goals every 20 seconds")
        
        # Pubblica subito il primo goal
        time.sleep(2.0)  # Aspetta che Nav2 sia ready
        self.publish_next_goal()

    def publish_next_goal(self):
        """Pubblica il prossimo goal nella sequenza"""
        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info("✅ All goals published! Stopping.")
            self.timer.cancel()
            return
        
        # Prendi goal corrente
        goal_x, goal_y = self.goals[self.current_goal_idx]
        
        # Crea messaggio
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # Orientamento neutro
        
        # Pubblica
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info(
            f"📡 Published Goal {self.current_goal_idx + 1}/{len(self.goals)}: "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )
        
        # Prossimo goal
        self.current_goal_idx += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleGoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

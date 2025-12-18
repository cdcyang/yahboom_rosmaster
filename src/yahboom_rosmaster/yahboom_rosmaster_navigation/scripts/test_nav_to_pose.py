#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yahboom_rosmaster_navigation.posestamped_msg_generator import PoseStampedGenerator

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose/goal', 10)
        self.get_logger().info('Goal Publisher node has been initialized')

        self.pose_generator = PoseStampedGenerator('pose_generator')
        self.locations = {
            '1': {'x': -0.96, 'y': -0.92},   # Table 1 location
            '2': {'x': 1.16, 'y': -4.23},    # Table 2 location
            '3': {'x': 0.792, 'y': -8.27},   # Table 3 location
            '4': {'x': -3.12, 'y': -7.495},  # Table 4 location
            '5': {'x': -2.45, 'y': -3.55}    # Table 5 location
        }

    def publish_goal(self, table_number):
        x = self.locations[table_number]['x']
        y = self.locations[table_number]['y']

        pose_msg = self.pose_generator.create_pose_stamped(
            x=x,
            y=y,
            z=0.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            frame_id='map'
        )

        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Goal pose published for table {table_number}')

    def run_interface(self):
        while True:
            print("\nAVAILABLE TABLES:")
            for table in self.locations:
                print(f"Table {table}")
            user_input = input('\nEnter table number (1-5) or "q" to quit: ').strip()
            if user_input.lower() == 'q':
                break
            if user_input in self.locations:
                self.publish_goal(user_input)
            else:
                print("Invalid table number! Please enter a number between 1 and 5.")


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()

    try:
        goal_publisher.run_interface()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up
        goal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

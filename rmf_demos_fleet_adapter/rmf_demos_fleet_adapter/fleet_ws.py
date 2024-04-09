import asyncio
import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_fleet_msgs.msg import RobotState, RobotMode, Location
from rmf_fleet_msgs.msg import PathRequest
from rclpy.qos import qos_profile_system_default
import threading
import json
import requests
import rmf_adapter as adpt
import sys
import argparse
import yaml
import math

from .RobotModel import RobotModel

class WebSocketNode(Node):
    def __init__(self, config):
        super().__init__('websocket_node')
        
        self.fleet_name = config['rmf_fleet']['name']
        self.robots = self.generate_robot_model(config['rmf_fleet']['robots'])
        self.get_logger().info(f'Robots: {self.robots}')
        self.create_subscription(PathRequest, 'robot_path_requests', self.task_callback, 10)
    
    def generate_robot_model(self, config):
        robots = {}
        for robot_name, value in config.items():
            robot = RobotModel(value['ip'], self.fleet_name, robot_name, self)
            robots[robot_name] = robot
        return robots


    def run(self):
        loop = asyncio.get_event_loop()
        tasks = []
        for robot_name, value in self.robots.items():
            uri = 'ws://{}:1234/robot_data'.format(value.ip)
            task = loop.create_task(value.start(uri=uri))
            tasks.append(task)
        tasks = asyncio.gather(*tasks)
        loop.run_until_complete(tasks)   

    
    def task_callback(self, msg):
        self.get_logger().info(f"Received task request: {msg}")
        # self.get_logger().info(f'navigation: path_request.task_id: {path_request.task_id}')
        self.robots[msg.robot_name].set_path_remaining(msg.path, task_id=msg.task_id)

        
def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')

def main(argv=sys.argv):
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the fleet ws node',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    args = parser.parse_args(args_without_ros[1:])
    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)

    websocket_node = WebSocketNode(config)
    spin_thread = threading.Thread(target=ros2_thread, args=(websocket_node,))
    spin_thread.start()
    websocket_node.run()  # replace with your WebSocket server URIs


    websocket_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
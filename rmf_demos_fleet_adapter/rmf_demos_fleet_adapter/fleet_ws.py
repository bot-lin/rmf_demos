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

class WebSocketNode(Node):
    def __init__(self, config):
        super().__init__('websocket_node')
        self.robot_state_publisher_ = self.create_publisher(RobotState, 'robot_state', 10)
        self.create_subscription(PathRequest, 'robot_path_requests', self.task_callback, 10)
        self.websockets = []
        self.fleet_name = config['rmf_fleet']['name']
        self.robots = config['rmf_fleet']['robots'].items()
        self.get_logger().info(f'Fleet name: {self.fleet_name}')
        self.get_logger().info(f'Robots: {self.robots}')
        self.tasks = {
            'tinyrobot1': []
        }
        self.latest_task_id = {
            'tinyrobot1': ''
        }

        self.original_x = -22.9
        self.original_y = -26.6
        self.height = 1021

    async def connect_to_websocket(self, uri):
        websocket = await websockets.connect(uri)
        self.websockets.append(websocket)

    async def start(self, robot_name, fleet_name, uri):
        await self.connect_to_websocket(uri)
        seq = 0
        while True:
            seq += 1
            for websocket in self.websockets:
                message = await websocket.recv()
                data_dict = json.loads(message)
                data_ros = RobotState()
                data_ros.name = 'tinyrobot1'
                data_ros.model = 'tinyrobot'
                data_ros.task_id = self.latest_task_id[data_ros.name]
                data_ros.seq = seq 
                data_ros.mode.mode = 0
                x, y = self.find_map_in_rmf(float(data_dict['pose']['position']['x']), float(data_dict['pose']['position']['y']), origin_x=self.original_x, origin_y=self.original_y, height=self.height)
                data_ros.battery_percent = float(data_dict['battery'])
                data_ros.location.x = x
                data_ros.location.y = y
                data_ros.location.yaw = float(data_dict['pose']['pyr']['yaw'])
                data_ros.location.level_name = 'L1'
                match data_dict['robot_mode']:
                    case 'idle': data_ros.mode.mode = 0
                    case 'charging': data_ros.mode.mode = 1
                    case 'moving': data_ros.mode.mode = 2
                    case 'paused': data_ros.mode.mode = 3
                    case 'waiting': data_ros.mode.mode = 4
                data_ros.location.t = self.get_clock().now().to_msg()
                self.robot_state_publisher_.publish(data_ros)
                self.confirm_robot_state(data_dict, data_ros.name)
                if data_ros.mode.mode in [0, 1] and len(self.tasks[data_ros.name]) > 0:
                    post_data = self.tasks[data_ros.name][0]
                    http_response = requests.post('http://10.6.75.222:1234/go_to', json=post_data)
                    self.get_logger().info(http_response.text)
                    if json.loads(http_response.text)["code"] == 0:
                        self.tasks[data_ros.name].pop(0)


    def confirm_robot_state(self, data_dict, robot_name):
        if data_dict['fsm'] in ['succeeded', 'canceled', 'failed']:
            http_response = requests.get('http://10.6.75.222:1234/confirm_status')
            self.get_logger().info(http_response.text)

    def run(self):
        loop = asyncio.get_event_loop()
        tasks = []
        for robot_name, value in self.robots:
            uri = 'ws://{}/robot_data'.format(value['ip'])
            task = loop.create_task(self.start(robot_name=robot_name, fleet_name=self.fleet_name, uri=uri))
            tasks.append(task)
        tasks = asyncio.gather(*tasks)
        loop.run_until_complete(tasks)
        
    def find_map_in_rmf(self, given_x, given_y, resolution=0.05, origin_x=-24.5, origin_y=-28.9, height=896):
        temp_x = given_x - origin_x
        temp_y = given_y - origin_y
        map_x = int(temp_x / resolution)
        map_y = int(temp_y / resolution)
        temp_y = height - map_y
        x = map_x * resolution
        y = - temp_y * resolution
        return x, y
    
    def find_map_in_ros(self, given_x, given_y, resolution=0.05, origin_x=-24.5, origin_y=-28.9, height=896):
        x = given_x + origin_x
        map_y = - given_y / resolution
        temp_y = height - map_y
        y = temp_y * resolution + origin_y
        return x, y
    
    def task_callback(self, msg):
        self.get_logger().info(f"Received task request: {msg}")
        # self.get_logger().info(f'navigation: path_request.task_id: {path_request.task_id}')
        target_x = msg.path[1].x
        target_y = msg.path[1].y
        target_yaw = msg.path[1].yaw
        map_x, map_y = self.find_map_in_ros(target_x, target_y, origin_x=self.original_x, origin_y=self.original_y, height=self.height)
        post_data = {
            "pose": {
                "position": {
                    "x": map_x,
                    "y": map_y
                },
                "pyr": {
                    "yaw": -target_yaw
                },
                "orientation": {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                }
            },
            "use_pyr": True,
            "precision_xy": 0.1,
            "precision_yaw": 0.1,
            "is_reverse": False,
            "nav_type": "auto",
            "task_id": str(msg.task_id),
            "inflation_radius": 1.1
        }
        self.tasks[msg.robot_name].append(post_data)
        self.latest_task_id[msg.robot_name] = str(msg.task_id)
        
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
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
class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.robot_state_publisher_ = self.create_publisher(RobotState, 'robot_state', 10)
        self.create_subscription(PathRequest, 'robot_path_requests', self.task_callback, 10)
        self.websockets = []

    async def connect_to_websocket(self, uri):
        websocket = await websockets.connect(uri)
        self.websockets.append(websocket)

    async def start(self, uri):
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
                data_ros.task_id = data_dict['current_task_uid']
                data_ros.seq = seq 
                data_ros.mode.mode = 0
                x, y = self.find_map_in_rmf(float(data_dict['pose']['position']['x']), float(data_dict['pose']['position']['y']))
                data_ros.battery_percent = float(data_dict['battery'])
                data_ros.location.x = x
                data_ros.location.y = y
                data_ros.location.yaw = float(data_dict['pose']['pyr']['yaw'])
                data_ros.location.level_name = 'L1'
                data_ros.location.t = self.get_clock().now().to_msg()
                self.robot_state_publisher_.publish(data_ros)

                msg = String()
                msg.data = f"{message}"
                self.publisher_.publish(msg)

    def run(self, uris):
        loop = asyncio.get_event_loop()
        tasks = []
        for uri in uris:
            task = loop.create_task(self.start(uri))
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
        map_x, map_y = self.find_map_in_a(target_x, target_y)
        post_data = {
            "pose": {
                "position": {
                    "x": map_x,
                    "y": map_y
                },
                "pyr": {
                    "yaw": target_yaw
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
        http_response = requests.post('http://10.6.75.222:1234/go_to', json=post_data)
        self.get_logger().info(http_response.text)
        # x = msg.path[0].x
        # y = msg.path[0].y
        # yaw = msg.path[0].yaw
        # post_data = {
        #     "pose": {
        #         "position": {
        #             "x": x,
        #             "y": y
        #         },
        #         "pyr": {
        #             "yaw": yaw
        #         }
        #     },
        #     "use_pyr": True,
        #     "precision_xy": 0.1,
        #     "precision_yaw": 0.1,
        #     "is_reverse": False,
        #     "nav_type": "auto",
        #     "task_id": msg.task_id,
        #     "inflation_radius": 1.1
        # }
        # response = requests.post('http://10.6.75.222:1234/go_to', json=post_data)
        # self.get_logger().info(response.text)
                      

def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')

def main(args=None):
    rclpy.init(args=args)

    websocket_node = WebSocketNode()
    spin_thread = threading.Thread(target=ros2_thread, args=(websocket_node,))
    spin_thread.start()
    websocket_node.run(['ws://10.6.75.222:1234/robot_data'])  # replace with your WebSocket server URIs


    websocket_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import requests
import json
import websockets
from rmf_fleet_msgs.msg import RobotState

class RobotModel:
    def __init__(self, ip, fleet_name, robot_name, node):
        self.path_remaining = []
        self.task_id = ''
        self.connected = False
        self.pose = None
        self.fleet_name = fleet_name
        self.robot_name = robot_name
        self.ip = ip
        self.node = node    
        self.robot_state_publisher_ = self.node.create_publisher(RobotState, 'robot_state', 10)
        self.get_map_info()
        self.set_robot_fleet_name()
    
    def __str__(self):
        return f'RobotModel: {self.__dict__}'
    
    def set_path_remaining(self, path, task_id):
        self.task_id = task_id
        self.path_remaining.append(path)
        self.post_dest_to_robot()

    def get_map_info(self):
        try:
            http_response = requests.get('http://{}/get_map_info'.format(self.ip))
            map_info = json.loads(http_response.text)['data']
            self.original_x = map_info['origin']['position']['x']
            self.original_y = map_info['origin']['position']['y']
            self.height = map_info['height']
            self.connected = True
        except:
            self.connected = False

    def set_robot_fleet_name(self):
        try:
            post_data = {
                "robot_name": self.robot_name,
                "fleet_name": self.fleet_name
            }
            http_response = requests.post('http://{}/set_fleet_name'.format(self.ip), json=post_data)
            self.get_logger().info(http_response.text)
        except:
            self.connected = False

    def confirm_robot_state(self, data_dict, robot_name):
        if data_dict['fsm'] in ['succeeded', 'canceled', 'failed']:
            http_response = requests.get('http://{}/confirm_status'.format(self.ip))
            self.node.get_logger().info(http_response.text)
    
    def find_map_in_ros(self, given_x, given_y, resolution=0.05, origin_x=-24.5, origin_y=-28.9, height=896):
        x = given_x + origin_x
        map_y = - given_y / resolution
        temp_y = height - map_y
        y = temp_y * resolution + origin_y
        return x, y
    
    def find_map_in_rmf(self, given_x, given_y, resolution=0.05, origin_x=-24.5, origin_y=-28.9, height=896):
        temp_x = given_x - origin_x
        temp_y = given_y - origin_y
        map_x = int(temp_x / resolution)
        map_y = int(temp_y / resolution)
        temp_y = height - map_y
        x = map_x * resolution
        y = - temp_y * resolution
        return x, y
    
    def post_dest_to_robot(self):
        target_x = self.path_remaining[0].x
        target_y = self.path_remaining[0].y
        target_yaw = self.path_remaining[0].yaw
        map_x, map_y = self.find_map_in_ros(target_x, target_y, origin_x=self.original_x, origin_y=self.original_y, height=self.height)
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
            "task_id": str(self.task_id),
            "inflation_radius": 1.1
        }
        http_response = requests.post('http://{}/go_to_simple'.format(self.ip), json=post_data)
        self.node.get_logger().info(http_response.text)
        if json.loads(http_response.text)["code"] == 0:
            pass

    async def start(self, uri):
        # await self.connect_to_websocket(uri)
        websocket = await websockets.connect(uri)
        seq = 0
        while True:
            seq += 1
            message = await websocket.recv()
            data_dict = json.loads(message)
            data_ros = RobotState()
            data_ros.name = data_dict['robot_name']
            data_ros.model = data_dict['fleet_name']
            data_ros.task_id = self.task_id
            data_ros.seq = seq 
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
            data_ros.location.t = self.node.get_clock().now().to_msg()
            data_ros.path = self.path_remaining
            
            # self.confirm_robot_state(data_dict, data_ros.name)
            # if data_ros.mode.mode in [0, 1] and len(self.tasks[data_ros.name]) > 0:
            #     self.robot_current_path[data_ros.name] = None
            #     post_data = self.tasks[data_ros.name][0]
            #     http_response = requests.post('http://{}/go_to'.format(self.robots[data_ros.name]['ip']), json=post_data)
            #     self.get_logger().info(http_response.text)
            #     if json.loads(http_response.text)["code"] == 0:
            #         self.tasks[data_ros.name].pop(0)
            #         self.robot_current_path[data_ros.name] = self.robot_path[data_ros.name].pop(0)
            # if self.robot_current_path[data_ros.name] is not None:
            #     data_ros.path.append(self.robot_current_path[data_ros.name])
            self.robot_state_publisher_.publish(data_ros)


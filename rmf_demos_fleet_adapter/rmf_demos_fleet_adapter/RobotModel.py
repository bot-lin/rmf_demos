import requests
import json
import websockets
import asyncio
import math
from rmf_fleet_msgs.msg import RobotState
from threading import Lock

class RobotModel:
    def __init__(self, ip, fleet_name, robot_name, node):
        self.path_remaining = []
        self.path_remaining_lock = Lock() 
        self.last_post_path = None
        self.task_id = ''
        self.action_type = ''
        self.robot_message = ''
        self.connected = False
        self.pose = None
        self.fleet_name = fleet_name
        self.robot_name = robot_name
        self.ip = ip
        self.node = node   
        self.waiting_for_zone = False
        self.resending_path = False
        self.robot_status = 'idle'
        self.zone_manager = node.zone_manager 
        self.robot_state_publisher_ = self.node.create_publisher(RobotState, 'robot_state', 10)
        self.robot_message_publisher_ = self.node.create_publisher(RobotState, 'robot_message', 10)
        self.last_pub_data = self.init_ros_data()
        self.last_pose_time = self.node.get_clock().now()
        self.node.create_timer(0.1, self.timer_callback)
        self.node.create_timer(1.0, self.post_dest_to_robot_callback)
        
    def post_dest_to_robot_callback(self):
        if self.close_enough_to_goal(self.last_pub_data.location.x, self.last_pub_data.location.y) or self.robot_status in ['succeeded', 'canceled', 'failed']:
            if len(self.path_remaining) > 0 and (not self.waiting_for_zone) and (not self.resending_path):
                tmp = self.path_remaining.pop(0)
            self.confirm_robot_state(self.robot_status)
        current_time = self.node.get_clock().now()
        if len(self.path_remaining) > 0  and (self.last_post_path != self.path_remaining[0][0] or (current_time - self.last_pose_time).nanoseconds / 1e9 > 60.0):
            with self.path_remaining_lock:
                self.post_dest_to_robot()
                self.last_pose_time = self.node.get_clock().now()
    
    def timer_callback(self):
        self.last_pub_data.location.t = self.node.get_clock().now().to_msg()
        self.robot_state_publisher_.publish(self.last_pub_data)
        data = RobotState()
        data.name = self.robot_name
        data.model = self.fleet_name
        data.task_id = self.robot_message
        self.robot_message_publisher_.publish(data)
        robot_x = round(self.last_pub_data.location.x, 2)
        robot_y = round(self.last_pub_data.location.y, 2)
        if self.last_post_path is not None:
            target_x = round(self.last_post_path.x, 2)
            target_y = round(self.last_post_path.y, 2)
            self.zone_manager.update_zone_polygon(self.robot_name, [robot_x, robot_y], [target_x, target_y])
        else:
            self.zone_manager.update_zone_polygon(self.robot_name, [robot_x, robot_y])
        self.zone_manager.pub_zones()
        # self.zone_manager.draw_zones()
    def init_ros_data(self):
        data = RobotState()
        data.name = self.robot_name
        data.model = self.fleet_name
        data.battery_percent = 0.0
        data.location.x = 91.0
        data.location.y = -44.0
        data.location.yaw = 0.1
        data.location.level_name = 'L1'
        data.mode.mode = 2
        data.location.t = self.node.get_clock().now().to_msg()
        data.path = []
        data.task_id = ''
        return data
    
    def __str__(self):
        return f'RobotModel: {self.__dict__}'
    
    def path_request_valid(self, path, task_id):
        if len(path) == 0:
            return False
        # if task_id == self.task_id:
        #     return False
        return True
    
    def set_path_remaining(self, path, task_id):
        self.node.get_logger().info("---------------------------")
        self.node.get_logger().info("Robot: {} \n".format(self.robot_name) + "Path request: {} \n".format(path) + "Task id: {}\n".format(task_id))
        if self.path_request_valid(path, task_id):
            self.node.get_logger().info("Path is Valid")
            self.task_id = task_id
            with self.path_remaining_lock:
                self.path_remaining = []
                length = self.get_path_length(path)
                self.path_remaining.append((path[-1], length))
            if path[0] == path[-1]:
                self.node.get_logger().info("################")
                self.node.get_logger().info("# Stop robot   #")
                self.stop_robot()
            
    def get_path_length(self, path):
        start = path[0]
        end = path[-1]
        return ((start.x - end.x)**2 + (start.y - end.y)**2)**0.5
    
    def start_nest_action(self, action_id, cmd_id):
        self.robot_message = "executing nest action: {}".format(action_id) 
        self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Sending nest action: {} ".format(action_id))
        http_response = requests.get('http://{}:5000/deploy/executeAction/{}/{}'.format(self.ip, action_id, cmd_id))
        if json.loads(http_response.text)["code"] == 0:
            self.last_post_path = self.path_remaining[0][0]
            self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Start nest action: {} ".format(action_id))
            self.resending_path = False
        else:
            self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Failed to start nest action: {} ".format(action_id))
            self.resending_path = True



    def get_map_info(self):
        try:
            http_response = requests.get('http://{}:1234/get_map_info'.format(self.ip), timeout=1)
            map_info = json.loads(http_response.text)['data']
            self.original_x = map_info['origin']['position']['x']
            self.original_y = map_info['origin']['position']['y']
            self.height = map_info['height']
            self.connected = True
        except:
            self.node.get_logger().info("{}: Failed to get map info".format(self.robot_name))
            self.connected = False

    def set_robot_fleet_name(self):
        try:
            post_data = {
                "robot_name": self.robot_name,
                "fleet_name": self.fleet_name
            }
            http_response = requests.post('http://{}:1234/set_fleet_name'.format(self.ip), json=post_data, timeout=1)
            self.node.get_logger().info(http_response.text)
        except:
            self.node.get_logger().info("{}: Failed to set fleet name".format(self.robot_name))
            self.connected = False

    def confirm_robot_state(self, robot_status):
        if robot_status in ['succeeded', 'canceled', 'failed']:
            http_response = requests.get('http://{}:1234/confirm_status'.format(self.ip))
            self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Confirm status response: {} ".format(http_response.text))
    
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

    def stop_robot(self):
        http_response = requests.get('http://{}:1234/stop_robot'.format(self.ip))
        self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Stop robot response: {} ".format(http_response.text))
        self.robot_message = "Robot stopped"
        self.path_remaining = []

    
    def post_dest_to_robot(self):
        
        if self.path_remaining[0][0].level_name == 'run_nest_action':
            self.start_nest_action(self.path_remaining[0][0].index, self.task_id)
            return
        target_x = self.path_remaining[0][0].x
        target_y = self.path_remaining[0][0].y
        target_yaw = self.path_remaining[0][0].yaw
        precision_xy = 0.1
        precision_yaw = 6.05
        if self.path_remaining[0][0].is_reverse:
            target_yaw += math.pi
            if target_yaw > math.pi:
                target_yaw -= 2 * math.pi
        if self.path_remaining[0][0].level_name == "go_to":
            self.action_type = "go_to_xy"
            precision_xy = 0.05
            precision_yaw = 0.05
            map_x, map_y = self.path_remaining[0][0].map_x, self.path_remaining[0][0].map_y
            # self.path_remaining[0][0].level_name = "L1"
            self.path_remaining[0][0].x, self.path_remaining[0][0].y = self.find_map_in_rmf(map_x, map_y, origin_x=self.original_x, origin_y=self.original_y, height=self.height)
        else:
            self.action_type = "go_to_place"
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
            "precision_xy": precision_xy,
            "precision_yaw": precision_yaw,
            "is_reverse": self.path_remaining[0][0].is_reverse,
            "nav_type": "auto",
            "task_id": str(self.task_id),
            "inflation_radius": 1.1
        }
        if not self.update_zone(self.robot_name, [self.path_remaining[0][0].x, self.path_remaining[0][0].y]):
            return
 
        self.waiting_for_zone = False
        self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Posting destination to robot: {} ".format(post_data))
        http_response = requests.post('http://{}:1234/go_to_simple'.format(self.ip), json=post_data)
        if json.loads(http_response.text)["code"] == 0:
            print_string = "Robot: {} ".format(self.robot_name) + "Go to simple response: {} ".format(http_response.text) + "map_x: {}, map_y: {} ".format(map_x, map_y) + "target_x: {}, target_y: {}".format(target_x, target_y)
            self.node.get_logger().info(print_string)
            self.last_post_path = self.path_remaining[0][0]
            self.resending_path = False
            self.robot_message = "Going to ({}, {})".format(map_x, map_y)
        else:
            self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Failed to post destination to robot: {} ".format(http_response.text))
            self.robot_message = "Failed to post go to ({}, {})".format(map_x, map_y)
            self.resending_path = True


    def update_zone(self, robot_name, target):
        is_in_zone, zone_name = self.zone_manager.is_point_in_zone(target)
        if is_in_zone:
            if self.zone_manager.allowed_to_enter_zone(zone_name=zone_name, robot_name=robot_name):
                self.zone_manager.add_vehicle_to_zone(robot_name, zone_name)
                return True
            else:
                self.node.get_logger().info(
                    f'Robot {robot_name} is not allowed to enter zone '
                    f'{zone_name} yet, waiting...'
                )
                vehicle_in_zone = self.zone_manager.get_vehicle_in_zone(
                    zone_name=zone_name
                )
                self.node.get_logger().info(
                    f'Since Vehicles in zone {zone_name}: {vehicle_in_zone}'
                )
                self.waiting_for_zone = True
                return False
        else:
            #check if leaving zone
            is_in_zone, zone_name = self.zone_manager.check_vehicle_in_zone(robot_name=robot_name)
            if is_in_zone:
                #leave zone
                self.zone_manager.vehicle_leave_zone(robot_name=robot_name, zone_name=zone_name)
        return True

    def close_enough_to_goal(self, x1, y1):
        if len(self.path_remaining) > 0:
            x2 = self.path_remaining[0][0].x
            y2 = self.path_remaining[0][0].y
            distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
            threshold = self.path_remaining[0][1] / 2
            if threshold > 1.0:
                threshold = 1.0
            elif threshold < 0.4:
                threshold = 0.4
            if self.action_type == "go_to_xy":
                threshold = 0.1
            if distance < threshold:
                self.node.get_logger().info("Robot: {} ".format(self.robot_name) + "Close enough to goal ({}, {}) with distance: {}".format(x2, y2, distance))
                return True
        return False
    
    def check_robot_mode(self, data_dict):
        if self.waiting_for_zone:
            return 4
        if data_dict['cm'] == 1 and len(self.path_remaining) > 0:
            return 4
        eps = 0.01
        speed = data_dict['speed']
        stationary = abs(speed['linear']) < eps and abs(speed['angular']) < eps
        if len(self.path_remaining) == 0:
            return 0
        else:
            return 2


    async def start(self, uri):
        self.seq = 0
        while True:
            if self.connected:
                try:
                    async with websockets.connect(uri, ping_timeout=2) as websocket:
                        await self.websocket_handling_logic(websocket)
                except (websockets.exceptions.ConnectionClosedError, asyncio.exceptions.CancelledError) as e:
                    self.connected = False
                    self.node.get_logger().info(f"{self.robot_name} WebSocket connection closed: {e}. Retrying in 5 seconds...")
                    await asyncio.sleep(5)
                except Exception as e:
                    self.connected = False
                    self.node.get_logger().info(f"{self.robot_name} Unexpected error: {e}. Retrying in 5 seconds...")
                    await asyncio.sleep(5)
            else:
                self.get_map_info()
                self.set_robot_fleet_name()

            
            # if self.last_pub_data is not None:
            #     self.robot_state_publisher_.publish(self.last_pub_data)

    async def websocket_handling_logic(self, websocket):
        
        try:
            while True:
                self.seq += 1
                message = await websocket.recv()
                data_dict = json.loads(message)
                data_ros = RobotState()
                data_ros.name = data_dict['robot_name']
                data_ros.model = data_dict['fleet_name']
                
                data_ros.seq = self.seq 
                x, y = self.find_map_in_rmf(float(data_dict['pose']['position']['x']), float(data_dict['pose']['position']['y']), origin_x=self.original_x, origin_y=self.original_y, height=self.height)
                data_ros.battery_percent = 100.0 #float(data_dict['battery'])
                data_ros.location.x = x
                data_ros.location.y = y
                # data_ros.location.map_x = float(data_dict['pose']['position']['x'])
                # data_ros.location.map_y = float(data_dict['pose']['position']['y'])
                data_ros.location.yaw = float(data_dict['pose']['pyr']['yaw'])
                data_ros.location.level_name = 'L1'
                data_ros.mode.mode = self.check_robot_mode(data_dict)
                self.robot_status = data_dict['fsm']
        
                data_ros.location.t = self.node.get_clock().now().to_msg()
                # if self.close_enough_to_goal(x, y) or data_dict['fsm'] in ['succeeded', 'canceled', 'failed']:
                #     if len(self.path_remaining) > 0 and (not self.waiting_for_zone) and (not self.resending_path):
                #         tmp = self.path_remaining.pop(0)
                #     self.confirm_robot_state(data_dict)
                    
                    # self.task_id = ''
                data_ros.path = [path for path, length in self.path_remaining]
                data_ros.task_id = self.task_id
            
                # self.robot_state_publisher_.publish(data_ros)
                self.last_pub_data = data_ros
                
        except asyncio.CancelledError:
            self.node.get_logger().info("WebSocket handling task was cancelled")


    
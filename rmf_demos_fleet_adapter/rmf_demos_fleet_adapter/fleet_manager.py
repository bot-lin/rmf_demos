#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import copy
import json
import math
import sys
import threading
import time
from typing import Optional

from fastapi import FastAPI
import numpy as np
from pydantic import BaseModel
from pyproj import Transformer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter as adpt
from rclpy.executors import MultiThreadedExecutor

import rmf_adapter.geometry as geometry
import rmf_adapter.vehicletraits as traits
from rmf_fleet_msgs.msg import DockSummary
from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import PathRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import RobotState
from std_msgs.msg import String
from rmf_demos_fleet_adapter.fleet_zone import FleetZoneManager

import socketio
import uvicorn
import yaml
import requests

app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    activity: Optional[str] = None
    label: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None
    action_id: Optional[int] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:

    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = self.svy_transformer.transform(
            gps_json['lat'], gps_json['lon']
        )
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


class FleetManager(Node):

    def __init__(self, config, nav_path):
        self.debug = False
        self.config = config
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']
        

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get('reference_coordinates')
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get('offset')
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        super().__init__(f'{self.fleet_name}_fleet_manager')
        self.zone_manager = FleetZoneManager(self.config['zones'], self)

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():
            self.robots[robot_name] = State()
        assert len(self.robots) > 0

        profile = traits.Profile(
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['footprint']
            ),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']
            ),
        )
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']
            ),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']
            ),
            profile=profile,
        )
        self.vehicle_traits.differential.reversible = self.config['rmf_fleet'][
            'reversible'
        ]

        fleet_manager_config = self.config['fleet_manager']
        self.action_paths = fleet_manager_config.get('action_paths', {})
        self.sio = socketio.Client()

        @self.sio.on('/gps')
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot['robot_id']
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                self.get_logger().info(f'Malformed GPS Message!: {e}')

        if self.gps:
            while True:
                try:
                    self.sio.connect('http://0.0.0.0:8080')
                    break
                except Exception:
                    self.get_logger().info(
                        'Trying to connect to sio server at '
                        'http://0.0.0.0:8080..'
                    )
                    time.sleep(1)

        self.create_subscription(
            RobotState, 'robot_state', self.robot_state_cb, 100
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos,
        )

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default,
        )


        @app.get('/open-rmf/rmf_demos_fm/status/', response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {'data': {}, 'success': False, 'msg': ''}
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name)
                    )
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                # self.get_logger().info("{} {} {}".format(state.state.location.x, state.state.location.y, state.state.location.yaw))
                response['data'] = self.get_robot_state(state, robot_name)
                # self.get_logger().info("{}".format(response['data']))
            response['success'] = True
            # rclpy.spin_once(self, timeout_sec=0.1)
            return response

        @app.post('/open-rmf/rmf_demos_fm/navigate/', response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            self.get_logger().info(f'navigate: {robot_name}')
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots or len(dest.destination) < 1:
                return response
            

            robot = self.robots[robot_name]

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = dest.map_name
            target_speed_limit = dest.speed_limit

            target_x -= self.offset[0]
            target_y -= self.offset[1]

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            robot = self.robots[robot_name]
            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.yaw
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(
                disp / self.vehicle_traits.linear.nominal_velocity
            ) + int(
                abs(abs(cur_yaw) - abs(target_yaw))
                / self.vehicle_traits.rotational.nominal_velocity
            )
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = target_map
            target_loc.obey_approach_speed_limit = False
            target_loc.is_reverse = True
            if target_speed_limit is not None and target_speed_limit > 0.0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            # threading.Thread(target=self.pub_path_thread, args=(robot_name, path_request, [target_x, target_y])).start()
            self.path_pub.publish(path_request)
            if self.debug:
                print(f'Sending navigate request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/', response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            self.get_logger().info(f'stop robot: {robot_name}')
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)

            path_request.task_id = str(cmd_id)
            self.get_logger().info(f'stop robot: path_request.task_id: {path_request.task_id}')
            self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending stop request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = None

            response['success'] = True
            return response

        @app.get(
            '/open-rmf/rmf_demos_fm/action_paths/', response_model=Response
        )
        async def action_paths(activity: str, label: str):
            self.get_logger().info(f'action paths: {activity} {label}')
            response = {'success': False, 'msg': ''}
            if activity not in self.action_paths:
                return response

            if label not in self.action_paths[activity][label]:
                return response

            response['data'] = self.action_paths[activity][label]
            response['success'] = True
            return response

        @app.post(
            '/open-rmf/rmf_demos_fm/start_activity/', response_model=Response
        )
        async def start_activity(
            robot_name: str, cmd_id: int, request: Request
        ):
            self.get_logger().info(f'start activity: {robot_name}')
            response = {'success': False, 'msg': ''}
            if (
                robot_name not in self.robots
                
            ):
                return response

            robot = self.robots[robot_name]

            path_request = PathRequest()
            cur_loc = robot.state.location
            target_loc = Location()
            path_request.path.append(cur_loc)
            self.get_logger().info('--------------------------------------------')
            self.get_logger().info(f'action paths: {request.activity} {request.label}')

            if request.activity in ['nest_action']:
                action_id = request.label
                target_loc = Location()
                target_loc.x = cur_loc.x + 10.0
                target_loc.y = cur_loc.y + 10.0
                target_loc.level_name = "run_nest_action"
                target_loc.index = int(action_id)
                path_request.path.append(target_loc)
            elif request.activity in ['go_to']:
                desc = json.loads(request.label)
                target_loc = Location()
                target_loc.x = desc['position'][0]
                target_loc.y = desc['position'][1]
                target_loc.yaw = math.radians(desc['position'][2])
                target_loc.is_reverse = desc['is_reverse']
                target_loc.level_name = "go_to"
                path_request.path.append(target_loc)
            else:
                activity_path = self.action_paths[request.activity][request.label]
                map_name = activity_path['map_name']
                for wp in activity_path['path']:
                    target_loc = Location()
                    target_loc.x = wp[0]
                    target_loc.y = wp[1]
                    target_loc.yaw = wp[2]
                    target_loc.level_name = map_name
                    path_request.path.append(target_loc)

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            self.get_logger().info(f'action paths: path_request.task_id: {path_request.task_id}')
            self.path_pub.publish(path_request)

            if self.debug:
                print(
                    f'Sending [{request.activity}] at [{request.label}] '
                    f'request for {robot_name}: {cmd_id}'
                )
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            response['data'] = {}
            try:
                response['data']['path'] = activity_path
            except:
                response['data']['path'] = []
            return response

        @app.post(
            '/open-rmf/rmf_demos_fm/toggle_teleop/', response_model=Response
        )
        async def toggle_teleop(robot_name: str, mode: Request):
            self.get_logger().info(f'toggle teleop: {robot_name}')
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response
            # Toggle action mode
            self.robots[robot_name].mode_teleop = mode.toggle
            response['success'] = True
            return response
        
    def pub_path_thread(self, robot_name, path_request, target):
        is_in_zone, zone_name = self.zone_manager.is_point_in_zone(
            target
        )
        if is_in_zone:
            while not self.zone_manager.allowed_to_enter_zone(
                zone_name=zone_name,
                robot_name=robot_name
            ):
                self.get_logger().info(
                    f'Robot {robot_name} is not allowed to enter zone '
                    f'{zone_name} yet, waiting...'
                )
                vehicle_in_zone = self.zone_manager.get_vehicle_in_zone(
                    zone_name=zone_name
                )
                self.get_logger().info(
                    f'Since Vehicles in zone {zone_name}: {vehicle_in_zone}'
                )
                time.sleep(1)
            self.zone_manager.add_vehicle_to_zone(
                robot_name, zone_name
            )
        else:
            #check if leaving zone
            is_in_zone, zone_name = self.zone_manager.check_vehicle_in_zone(robot_name=robot_name)
            if is_in_zone:
                #leave zone
                self.zone_manager.vehicle_leave_zone(robot_name=robot_name, zone_name=zone_name)
        self.path_pub.publish(path_request)


    def robot_state_cb(self, msg):
        
        if msg.name in self.robots:
            robot = self.robots[msg.name]
            if (
                not robot.is_expected_task_id(msg.task_id)
                and not robot.mode_teleop
            ):
                # This message is out of date, so disregard it.
                if robot.last_path_request is not None:
                    # Resend the latest task request for this robot, in case
                    # the message was dropped.
                    if self.debug:
                        print(
                            f'Republishing task request for {msg.name}: '
                            f'{robot.last_path_request.task_id}, '
                            f'because it is currently following {msg.task_id}'
                        )
                    # self.path_pub.publish(robot.last_path_request)
                return

            robot.state = msg
            # self.get_logger().info("callback: {}".format(robot.state.location.x))
            
            # Check if robot has reached destination
            if robot.destination is None:
                return

            if (
                msg.mode.mode == RobotMode.MODE_IDLE
                or msg.mode.mode == RobotMode.MODE_CHARGING
            ) and len(msg.path) == 0:
                robot = self.robots[msg.name]
                robot.destination = None
                completed_request = int(msg.task_id)
                if robot.last_completed_request != completed_request:
                    if self.debug:
                        print(
                            f'Detecting completed request for {msg.name}: '
                            f'{completed_request}'
                        )
                robot.last_completed_request = completed_request

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if fleet.fleet_name == self.fleet_name:
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(robot.gps_pos)
        else:
            position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] = {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (
            robot.destination is not None
            and robot.last_path_request is not None
        ):
            destination = robot.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target = self.disp(
                position, [destination.x, destination.y]
            )
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (
                dist_to_target / self.vehicle_traits.linear.nominal_velocity
                + ori_delta / self.vehicle_traits.rotational.nominal_velocity
            )
            cmd_id = int(robot.last_path_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration,
            }
        else:
            data['destination_arrival'] = None

        data['last_completed_request'] = robot.last_completed_request
        if (
            robot.state.mode.mode == RobotMode.MODE_WAITING
            or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        ):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False

        return data

    def disp(self, A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def ros2_thread(node):
    print('entering ros2 thread')
    node.spin()
    print('leaving ros2 thread')

def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the fleet adapter',
    )
    parser.add_argument(
        '-c',
        '--config_file',
        type=str,
        required=True,
        help='Path to the config.yaml file',
    )
    parser.add_argument(
        '-n',
        '--nav_graph',
        type=str,
        required=True,
        help='Path to the nav_graph for this fleet adapter',
    )
    args = parser.parse_args(args_without_ros[1:])
    print('Starting fleet manager...')

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fleet_manager)    

    spin_thread = threading.Thread(target=ros2_thread, args=(executor,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config['fleet_manager']['ip'],
        port=config['fleet_manager']['port'],
        log_level='warning',
    )


if __name__ == '__main__':
    main(sys.argv)

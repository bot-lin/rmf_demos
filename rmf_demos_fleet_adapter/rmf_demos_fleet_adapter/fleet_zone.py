from typing import Any
from shapely.geometry import Point, Polygon
from std_msgs.msg import String
# import matplotlib.pyplot as plt

import math
import json
class FleetZone:
    def __init__(self, points=[], radius=1.0, max_vehicles=1, type="Polygon", vehicles=[]):
        self.type = type
        if type == "Polygon":
            self.zone = Polygon(points)
        elif type == "Circle":
            self.center = (0.0, 0.0)
            self.radius = radius
        self.vehicles = vehicles
        self.max_vehicles = max_vehicles
    
    def check_point_in_zone(self, point):
        if self.type == "Polygon":
            return self.zone.contains(Point(point[0], point[1]))
        elif self.type == "Circle":
            return self.__is_point_in_circle(self.center, self.radius, point)
        
    def update_center(self, center):
        self.center = center
    
    def update_zone_polygon(self, robot_points, tartget_points=None):
        if tartget_points is None:
            self.zone = Polygon(self.find_square_points(robot_points[0], robot_points[1], 1.5))
        else:
            self.zone = Polygon(self.find_rectangle_points(robot_points[0], robot_points[1], tartget_points[0], tartget_points[1], 1.0, 1.5))    
    def find_square_points(self, x, y, d):
        # Compute the coordinates of the four points of the square
        A1_x = x - d
        A1_y = y - d

        A2_x = x + d
        A2_y = y - d

        B1_x = x + d
        B1_y = y + d

        B2_x = x - d
        B2_y = y + d

        return [(A1_x, A1_y), (A2_x, A2_y), (B1_x, B1_y), (B2_x, B2_y)]


    def find_rectangle_points(self, x1, y1, x2, y2, d, d2):
        # Step 1: Direction vector
        dir_x = x2 - x1
        dir_y = y2 - y1

        # Step 2: Normalize the direction vector
        length = math.sqrt(dir_x**2 + dir_y**2)
        if length < 1e-6:
            length = 0.1
        unit_x = dir_x / length
        unit_y = dir_y / length

        # Step 3: Perpendicular vector
        perp_x = -unit_y
        perp_y = unit_x

        # Step 4: Compute the rectangle points
        # Points near P1
        A1_x = x1 + d * perp_x - d2 * unit_x
        A1_y = y1 + d * perp_y - d2 * unit_y

        A2_x = x1 - d * perp_x - d2 * unit_x
        A2_y = y1 - d * perp_y - d2 * unit_y

        # Points near P2
        B1_x = x2 + d * perp_x + d2 * unit_x
        B1_y = y2 + d * perp_y + d2 * unit_y

        B2_x = x2 - d * perp_x + d2 * unit_x
        B2_y = y2 - d * perp_y + d2 * unit_y

        return [(A1_x, A1_y), (A2_x, A2_y), (B1_x, B1_y), (B2_x, B2_y)]

        
    def __is_point_in_circle(self, center, radius, point):
        distance = math.sqrt((center[0] - point[0]) ** 2 + (center[1] - point[1]) ** 2)
        return distance <= radius
    
    def __str__(self) -> str:
        match self.type:
            case "Circle":
                return f"Center: {self.center}, Radius: {self.radius}, Max Vehicles: {self.max_vehicles}, Vehicles: {self.vehicles}"
            case "Polygon":
                return f"Zone: {self.zone}, Max Vehicles: {self.max_vehicles}, Vehicles: {self.vehicles}"

 

class FleetZoneManager:
    def __init__(self, zone_config: dict, node_handler):
        self.node = node_handler
        self.node.get_logger().info('Initializing FleetZoneManager')
        self.node.get_logger().info('Zone Config: {}'.format(zone_config))
        self.zone_publiser = self.node.create_publisher(String, '/fleet_adapter/zones', 10)
        self.zones = {}
        for zone_name, zone in zone_config.items():
            if zone.get('type') == 'Circle':
                self.zones[zone_name] = FleetZone(radius=zone['radius'], max_vehicles=zone.get('max_vehicles', 1), type="Circle", vehicles=[zone_name])
            else:
                self.zones[zone_name] = FleetZone(zone.get('points', [[0.0, 0.0],[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]]), 
                                                  max_vehicles=zone.get('max_vehicles', 1), 
                                                  type="Polygon", 
                                                  vehicles=zone.get('vehicles', []))
        self.print_zones()

    def is_point_in_zone(self, point):
        for zone_name, zone in self.zones.items():
            if zone.check_point_in_zone(point):
                self.node.get_logger().info('Point: {} is in Zone: {}'.format(point, zone_name))
                return True, zone_name
        return False, zone_name
    
    def allowed_to_enter_zone(self, zone_name, robot_name):
        return len(self.zones[zone_name].vehicles) < self.zones[zone_name].max_vehicles or (robot_name in self.zones[zone_name].vehicles)
    
    def add_vehicle_to_zone(self, robot_name, zone_name):
        if robot_name not in self.zones[zone_name].vehicles:
            self.zones[zone_name].vehicles.append(robot_name)
            self.node.get_logger().info('Vehicle: {} entered Zone: {}'.format(robot_name, zone_name))

    def get_vehicle_in_zone(self, zone_name):
        return self.zones[zone_name].vehicles

    def vehicle_leave_zone(self, robot_name, zone_name):
        self.zones[zone_name].vehicles.remove(robot_name)
        self.node.get_logger().info('Vehicle: {} left Zone: {}'.format(robot_name, zone_name))
    
    def check_vehicle_in_zone(self, robot_name):
        for zone_name, zone in self.zones.items():
            if robot_name in zone.vehicles and zone_name != robot_name:
                return True, zone_name
        return False, zone_name   

    def update_zone_center(self, zone_name, center):
        self.zones[zone_name].update_center(center)
    
    def update_zone_polygon(self, zone_name, robot_points, tartget_points=None):
        self.zones[zone_name].update_zone_polygon(robot_points, tartget_points)

    def print_zones(self):
        for zone_name, zone in self.zones.items():
            self.node.get_logger().info('Zone: {}'.format(zone_name))
            self.node.get_logger().info('{}'.format(zone))
    
    def pub_zones(self):
        msg = String()
        msg.data = json.dumps({zone_name: str(zone) for zone_name, zone in self.zones.items()})
        self.zone_publiser.publish(msg)
    
    def draw_zones(self, filename="/data/polygons.png"):
        plt.figure()
        
        for name, zone in self.zones.items():
            x, y = zone.zone.exterior.xy
            plt.plot(x, y, label=name)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Polygons')
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(filename)
        plt.close()


        
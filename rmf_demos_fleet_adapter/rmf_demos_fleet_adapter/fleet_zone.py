from typing import Any
from shapely.geometry import Point, Polygon
import math

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
        
    def __is_point_in_circle(self, center, radius, point):
        distance = math.sqrt((center[0] - point[0]) ** 2 + (center[1] - point[1]) ** 2)
        return distance <= radius



class FleetZoneManager:
    def __init__(self, zone_config: dict, node_handler):
        self.node = node_handler
        self.node.get_logger().info('Initializing FleetZoneManager')
        self.node.get_logger().info('Zone Config: {}'.format(zone_config))
        self.zones = {}
        for zone_name, zone in zone_config.items():
            if zone.get('type') == 'Circle':
                self.zones[zone_name] = FleetZone(radius=zone['radius'], max_vehicles=zone.get('max_vehicles', 1), type="Circle")
            else:
                self.zones[zone_name] = FleetZone(zone['points'], max_vehicles=zone.get('max_vehicles', 1), type="Polygon")
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
            if robot_name in zone.vehicles:
                return True, zone_name
        return False, zone_name        

    def print_zones(self):
        for zone_name, zone in self.zones.items():
            self.node.get_logger().info('Zone: {}'.format(zone_name))
            self.node.get_logger().info('Points: {}'.format(zone.zone))
            self.node.get_logger().info('Max Vehicles: {}'.format(zone.max_vehicles))
   
  
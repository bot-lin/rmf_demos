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
import re

from .RobotModel import RobotModel
from rmf_demos_fleet_adapter.fleet_zone import FleetZoneManager
from std_msgs.msg import String

from shapely.geometry import Polygon
import matplotlib.pyplot as plt

def convert_polygon_str_to_array(polygon_str):
    # Use regular expression to extract coordinate pairs
    pattern = r'\(([^()]+)\)'
    matches = re.findall(pattern, polygon_str)
    
    # Convert the extracted strings to tuples of floats
    polygon_points = []
    for match in matches:
        # Split the coordinates by spaces and convert them to float
        points = match.split(',')
        for point in points:
            coordinates = point.strip().split()
            tuple_points = tuple(map(float, coordinates))
            polygon_points.append(tuple_points)
    
    return polygon_points

class PolygonPlotter(Node):
    def __init__(self):
        super().__init__('plot_show')
        self.create_subscription(String, '/fleet_adapter/zones', self.zone_callback, 10)
        self.polygons = {}
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(1.0, self.update_plot) 
        
    def zone_callback(self, msg):
        # Parse the message to extract polygon data
        # Here we assume the msg.data is a JSON string with polygon coordinates
        import json
        data = json.loads(msg.data)
        for name, coords in data.items():
            coords = convert_polygon_str_to_array(coords)
            self.get_logger().info(f'Got polygon {name}: {coords}')
            self.get_logger().info(f'Polygon type: {type(coords)}')
            self.polygons[name] = Polygon(coords)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(0, 110)
        self.ax.set_ylim(-50, 0)
        for name, polygon in self.polygons.items():
            x, y = polygon.exterior.xy
            self.ax.plot(x, y, label=name)
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)

    
        

def main(argv=sys.argv):
    rclpy.init(args=argv)
    adpt.init_rclcpp()

    plotter = PolygonPlotter()

    # Start the plot in interactive mode
    plt.ion()
    plt.show()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
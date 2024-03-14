import asyncio
import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_fleet_msgs.msg import RobotState, RobotMode, Location
import json
    
class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
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
                data_ros.name = 'robot_1'
                data_ros.model = 'diff'
                data_ros.task_id = ''
                data_ros.seq = seq 
                data_ros.mode.mode = 0
                data_ros.battery_percent = float(data_dict['battery'])
                data_ros.location.x = float(data_dict['pose']['position']['x'])
                data_ros.location.y = float(data_dict['pose']['position']['y'])
                data_ros.location.yaw = float(data_dict['pose']['pyr']['z'])
                data_ros.location.level_name = 'L1'
                data_ros.location.t = self.get_clock().now().to_msg()

                msg = String()
                msg.data = f"{message}"
                self.publisher_.publish(msg)

    def run(self, uris):
        for uri in uris:
            asyncio.get_event_loop().run_until_complete(self.start(uri))

def main(args=None):
    rclpy.init(args=args)

    websocket_node = WebSocketNode()
    websocket_node.run(['ws://10.6.75.222:1234/robot_data'])  # replace with your WebSocket server URIs

    rclpy.spin(websocket_node)

    websocket_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
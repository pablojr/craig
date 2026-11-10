#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import asyncio
import websockets
import json
import threading

class SpeechToSpeechVisemeBridge(Node):
    def __init__(self):
        super().__init__('s2s_viseme_bridge_node')
        
        self.jaw_pub = self.get_publisher(Float32, '/jaw', 10)
        
        self.declare_parameter('server_url', 'ws://192.168.0.202:8765/v1/realtime')
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.get_logger().info(f'Configured target S2S server URL: {self.server_url}')

        # Map viseme classifications to structural mouth openness positions (0.0 to 1.0)
        # Adjust these values depending on how wide your robot servo should throw
        self.viseme_jaw_map = {
            'silence': 0.0,
            'PP': 0.0,   # p, b, m (lips closed)
            'FF': 0.1,   # f, v
            'TH': 0.2,   # th
            'DD': 0.3,   # d, t, n
            'kk': 0.4,   # k, g
            'CH': 0.4,   # ch, j, sh
            'SS': 0.2,   # s, z
            'nn': 0.3,   # n
            'RR': 0.3,   # r
            'aa': 0.9,   # wide open vowels (a)
            'E':  0.6,   # e
            'I':  0.5,   # i
            'O':  0.8,   # o (rounded/open)
            'U':  0.4,   # u (tight circle)
        }
        
        self.bridge_thread = threading.Thread(target=self.start_websocket_loop, daemon=True)
        self.bridge_thread.start()
        self.get_logger().info('ROS 2 Viseme-to-Jaw Bridge Node Active.')

    def start_websocket_loop(self):
        asyncio.run(self.listen_to_s2s())

    async def listen_to_s2s(self):
        async for websocket in websockets.connect(self.server_url):
            try:
                async for message in websocket:
                    data = json.loads(message)
                    
                    # Intercept the specialized viseme event stream
                    if data.get('type') == 'viseme' or 'viseme' in data:
                        viseme_label = data.get('viseme') # e.g. "aa", "O", "PP"
                        
                        if viseme_label in self.viseme_jaw_map:
                            jaw_target = self.viseme_jaw_map[viseme_label]
                            self.publish_jaw(jaw_target)
                            
                    elif data.get('type') == 'speech.stopped':
                        self.publish_jaw(0.0)

            except websockets.ConnectionClosed:
                self.publish_jaw(0.0)
                await asyncio.sleep(2)
            except Exception as e:
                self.get_logger().error(f"Bridge error: {str(e)}")

    def publish_jaw(self, value):
        msg = Float32()
        msg.data = float(value)
        self.jaw_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToSpeechVisemeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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
        
        self.jaw_pub = self.create_publisher(Float32, '/jaw', 10)
        
        self.declare_parameter('server_url', 'ws://127.0.0.1:8765/v1/realtime')
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        
        self.get_logger().info(f'Configured target S2S server URL: {self.server_url}')
        
        self.viseme_jaw_map = {
            'silence': 0.0, 'PP': 0.0, 'FF': 0.1, 'TH': 0.2, 'DD': 0.3,
            'kk': 0.4, 'CH': 0.4, 'SS': 0.2, 'nn': 0.3, 'RR': 0.3,
            'aa': 0.9, 'E':  0.6, 'I':  0.5, 'O':  0.8, 'U':  0.4,
        }
        
        self.bridge_thread = threading.Thread(target=self.start_websocket_loop, daemon=True)
        self.bridge_thread.start()
        self.get_logger().info('ROS 2 Viseme-to-Jaw Bridge Node initialized.')

    def start_websocket_loop(self):
        asyncio.run(self.listen_to_s2s())

    async def listen_to_s2s(self):
        self.get_logger().info(f"Opening background connection to stream at: {self.server_url}")
        
        async for websocket in websockets.connect(self.server_url):
            try:
                self.get_logger().info("Connected to S2S Server!")
                async for message in websocket:
                    data = json.loads(message)
                    
                    # DEBUG LOG 1: Print raw keys of every message arriving from the server
                    self.get_logger().debug(f"Received JSON frame with keys: {list(data.keys())}")
                    
                    # Flexible check for the viseme payload structure
                    if data.get('type') == 'viseme' or 'viseme' in data:
                        viseme_label = data.get('viseme')
                        
                        # DEBUG LOG 2: Viseme detected! Log the label string
                        self.get_logger().info(f"Captured Viseme event from server: '{viseme_label}'")
                        
                        if viseme_label in self.viseme_jaw_map:
                            jaw_target = self.viseme_jaw_map[viseme_label]
                            self.publish_jaw(jaw_target)
                        else:
                            # DEBUG LOG 3: Warning if the server outputs a label we didn't map
                            self.get_logger().warn(f"Viseme '{viseme_label}' is missing from self.viseme_jaw_map!")
                            
                    elif data.get('type') == 'speech.stopped':
                        self.get_logger().info("Speech stopped event received. Resetting jaw.")
                        self.publish_jaw(0.0)

            except websockets.ConnectionClosed:
                self.get_logger().warn("Remote connection severed. Attempting recovery...")
                self.publish_jaw(0.0)
                await asyncio.sleep(2)
            except Exception as e:
                self.get_logger().error(f"WebSocket client loop exception occurred: {str(e)}")

    def publish_jaw(self, value):
        msg = Float32()
        msg.data = float(value)
        # DEBUG LOG 4: Confirm actual topic dispatch
        self.get_logger().info(f"Publishing to /jaw: {msg.data}")
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

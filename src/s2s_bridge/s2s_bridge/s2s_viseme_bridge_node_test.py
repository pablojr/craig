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
        
        # Microsoft Viseme IDs (0-21) mapped to jaw openness (0.0 to 1.0)
        self.ms_viseme_jaw_map = {
            0: 0.0,   # Silence
            1: 0.7,   # ae, ax, ah
            2: 0.9,   # aa (wide open)
            3: 0.8,   # ao
            4: 0.6,   # ey, eh, uh
            5: 0.5,   # er
            6: 0.4,   # y, iy, ih, ix
            7: 0.3,   # w, uw (rounded)
            8: 0.7,   # ow
            9: 0.8,   # aw
            10: 0.7,  # oy
            11: 0.8,  # ay
            12: 0.3,  # h
            13: 0.3,  # r
            14: 0.4,  # l
            15: 0.2,  # s, z
            16: 0.3,  # sh, ch, jh, zh
            17: 0.2,  # th, dh
            18: 0.1,  # f, v
            19: 0.3,  # d, t, n
            20: 0.4,  # k, g, ng
            21: 0.0,  # p, b, m (lips closed)
        }
        
        self.active_tasks = []
        self.bridge_thread = threading.Thread(target=self.start_websocket_loop, daemon=True)
        self.bridge_thread.start()
        self.get_logger().info(f'Viseme Bridge initialized. Target: {self.server_url}')

    def start_websocket_loop(self):
        asyncio.run(self.listen_to_s2s())

    async def listen_to_s2s(self):
        async for websocket in websockets.connect(self.server_url):
            try:
                self.get_logger().info("Connected to S2S Realtime WebSocket!")
                
                # Optional: Trigger a greeting so you can test immediately without a mic
                await self.send_test_prompt(websocket, "Hello! Testing my robot jaw movement.")

                async for message in websocket:
                    data = json.loads(message)
                    event_type = data.get('type', '')
                    
                    # Log every event type arriving on this session
                    self.get_logger().info(f"Event received: {event_type}")

                    if event_type == 'speech_to_speech.output_audio.visemes':
                        viseme_batch = data.get('visemes', [])
                        self.get_logger().info(f"Received batch of {len(viseme_batch)} visemes.")
                        
                        # Schedule the timed playback for this batch
                        task = asyncio.create_task(self.play_viseme_batch(viseme_batch))
                        self.active_tasks.append(task)

                    elif event_type in ['response.cancelled', 'conversation.item.truncated']:
                        self.get_logger().info("Response interrupted. Resetting jaw.")
                        self.cancel_scheduled_visemes()
                        self.publish_jaw(0.0)

            except websockets.ConnectionClosed:
                self.get_logger().warn("Connection closed. Reconnecting...")
                self.publish_jaw(0.0)
                await asyncio.sleep(2)
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {str(e)}")

    async def send_test_prompt(self, websocket, text):
        """Sends a text item to force the server to speak and generate visemes."""
        self.get_logger().info(f"Sending test text prompt: '{text}'")
        event = {
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [{"type": "input_text", "text": text}]
            }
        }
        await websocket.send(json.dumps(event))
        await websocket.send(json.dumps({"type": "response.create"}))

    async def play_viseme_batch(self, viseme_batch):
        """Plays through the viseme cues according to their start_s timestamps."""
        if not viseme_batch:
            return
            
        batch_start_offset = viseme_batch[0].get('start_s', 0.0)
        loop_start_time = asyncio.get_event_loop().time()

        for cue in viseme_batch:
            viseme_id = cue.get('viseme', 0)
            start_s = cue.get('start_s', 0.0)
            end_s = cue.get('end_s', start_s + 0.05)

            # Calculate delay relative to when this batch began
            relative_delay = max(0.0, (start_s - batch_start_offset) - (asyncio.get_event_loop().time() - loop_start_time))
            if relative_delay > 0:
                await asyncio.sleep(relative_delay)

            jaw_val = self.ms_viseme_jaw_map.get(viseme_id, 0.0)
            self.publish_jaw(jaw_val)

        # Hold the last viseme briefly, then close if it's the end
        last_cue = viseme_batch[-1]
        duration = max(0.0, last_cue.get('end_s', 0.0) - last_cue.get('start_s', 0.0))
        await asyncio.sleep(duration)
        self.publish_jaw(0.0)

    def cancel_scheduled_visemes(self):
        for task in self.active_tasks:
            if not task.done():
                task.cancel()
        self.active_tasks.clear()

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

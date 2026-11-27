"""InMoov perception node

- Subscribes to the webcam image (`/camera/image_raw`).
- Runs YOLO11n on CPU to track a target class.
- Publishes a normalized target pose on `/inmoov/head_target` (geometry_msgs/Point).
- Exposes a ROS string topic `/inmoov/vision_query` for free‑form visual questions.
- Sends the latest frame to the VLM (Ollama qwen2.5vl:7b) and publishes the concise answer on `/inmoov/vision_result`.

The node is fully self‑contained; adjust `VM1_OLLAMA_URL` to the address of your Ollama server.
"""

import base64
import threading
import cv2
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO

# Adjust this to point to the Ollama VLM instance on VM1
VM1_OLLAMA_URL = "http://<VM1_IP>:11434/v1/chat/completions"

class InMoovPerceptionNode(Node):
    def __init__(self):
        super().__init__('inmoov_perception_node')
        self.bridge = CvBridge()
        self.latest_bgr = None
        self.frame_lock = threading.Lock()
        self.target_class = "person"

        # Load YOLO11n model (CPU)
        self.detector = YOLO("yolo11n.pt")

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(String, '/inmoov/vision_query', self.query_cb, 10)
        self.create_subscription(String, '/inmoov/track_class', self.track_class_cb, 10)

        # Publishers
        self.target_pub = self.create_publisher(Point, '/inmoov/head_target', 10)
        self.result_pub = self.create_publisher(String, '/inmoov/vision_result', 10)

        # Run tracking at ~16 Hz
        self.create_timer(0.06, self.tracking_loop)

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def image_cb(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.frame_lock:
            self.latest_bgr = bgr

    def track_class_cb(self, msg: String):
        self.target_class = msg.data.strip().lower()
        self.get_logger().info(f"Tracking target switched to: {self.target_class}")

    def query_cb(self, msg: String):
        # Run the VLM query in a background thread so we don't block the ROS spinner
        threading.Thread(target=self._run_vlm, args=(msg.data,), daemon=True).start()

    # ---------------------------------------------------------------------
    # Tracking loop (runs at ~16 Hz)
    # ---------------------------------------------------------------------
    def tracking_loop(self):
        with self.frame_lock:
            if self.latest_bgr is None:
                return
            frame = self.latest_bgr.copy()

        h, w, _ = frame.shape
        results = self.detector.predict(frame, imgsz=416, device="cpu", verbose=False)

        best_box = None
        best_area = 0.0
        for r in results:
            for box in r.boxes:
                cls_name = self.detector.names[int(box.cls)].lower()
                conf = float(box.conf)
                if cls_name == self.target_class and conf > 0.45:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    area = (x2 - x1) * (y2 - y1)
                    if area > best_area:
                        best_area = area
                        best_box = (x1, y1, x2, y2)

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx = ((x1 + x2) / 2.0) / w
            cy = ((y1 + y2) / 2.0) / h
            pt = Point()
            pt.x = (cx - 0.5) * 2.0        # Normalized yaw [-1, 1]
            pt.y = (cy - 0.5) * 2.0        # Normalized pitch [-1, 1]
            pt.z = best_area / (w * h)     # Normalized area (proximity)
            self.target_pub.publish(pt)

    # ---------------------------------------------------------------------
    # VLM query handling
    # ---------------------------------------------------------------------
    def _run_vlm(self, prompt: str):
        # Guard against empty queries – the VLM needs non‑empty text.
        if not prompt or not prompt.strip():
            self._publish_result("Please ask a visual question.")
            return
        with self.frame_lock:
            if self.latest_bgr is None:
                self._publish_result("My camera feed is currently offline.")
                return
            frame = cv2.resize(self.latest_bgr, (640, 480))

        _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        b64_img = base64.b64encode(buf).decode('utf-8')

        payload = {
            "model": "qwen2.5vl:7b",
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": f"Answer concisely in one natural spoken sentence: {prompt}"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64_img}"}}
                ]
            }],
            "stream": False,
            "options": {"num_predict": 80, "temperature": 0.2}
        }
        try:
            r = requests.post(VM1_OLLAMA_URL, json=payload, timeout=15)
            r.raise_for_status()
            msg = r.json()["choices"][0]["message"]
            answer = msg.get("content", "").strip()
            # If the model returns a tool call without textual content, acknowledge it.
            if not answer and "tool_calls" in msg:
                answer = "[tool call received]"
            if not answer:
                answer = "I could not generate a response."
        except Exception as e:
            self.get_logger().error(f"Ollama VLM error: {e}")
            answer = "I had trouble processing what I see."
        self._publish_result(answer)

    def _publish_result(self, text: str):
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(InMoovPerceptionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

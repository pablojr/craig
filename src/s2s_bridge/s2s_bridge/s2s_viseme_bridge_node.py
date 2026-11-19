#!/usr/bin/env python3
"""ROS 2 WebSocket proxy for S2S viseme interception with smooth jaw control
and LLM-driven gesture triggering.

The S2S talk client connects to this node.  The node opens one upstream
connection to the S2S server, consumes timed viseme events to publish smooth
/jaw commands, accumulates the assistant transcript to select a gesture via
the rule-based classifier, and forwards every other WebSocket frame unchanged
in both directions.

Topics published
----------------
/jaw            std_msgs/Float64   – jaw position in radians (from visemes)
/robot/action   std_msgs/String    – JSON gesture command, e.g.
                                     {"action": "wave_right"}
"""

import asyncio
import json
import threading
from contextlib import suppress
from urllib.parse import urlsplit, urlunsplit

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

# Use the legacy API explicitly: it supports the one-argument handler and
# extra_headers interface used by this proxy across websockets releases.
try:
    from websockets.legacy.client import connect as websocket_connect
    from websockets.legacy.server import serve as websocket_serve
except ImportError:
    # Compatibility fallback for installations that expose only the
    # top-level API (older websockets releases).
    import websockets

    websocket_connect = websockets.connect
    websocket_serve = websockets.serve

from websockets.exceptions import ConnectionClosed

from .gesture_classifier import classify_with_confidence


class SpeechToSpeechVisemeBridge(Node):
    def __init__(self):
        super().__init__("s2s_viseme_bridge_node")

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self.jaw_pub = self.create_publisher(Float64, "/jaw", 10)
        self.action_pub = self.create_publisher(String, "/robot/action", 10)

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter("server_url", "ws://192.168.0.202:8765/v1/realtime")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 18765)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("attack_tau_s", 0.045)
        self.declare_parameter("release_tau_s", 0.110)
        self.declare_parameter("jaw_min", 0.0)
        self.declare_parameter("jaw_max", 0.349)
        self.declare_parameter("publish_epsilon", 0.002)

        # Gesture parameters
        self.declare_parameter("gesture_enabled", True)
        self.declare_parameter("gesture_suppress_neutral", True)  # don't publish "neutral"
        self.declare_parameter("gesture_min_text_length", 10)     # chars before classifying

        self.server_url = self.get_parameter("server_url").get_parameter_value().string_value
        self.listen_host = self.get_parameter("listen_host").get_parameter_value().string_value
        self.listen_port = self.get_parameter("listen_port").get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.attack_tau_s = self.get_parameter("attack_tau_s").get_parameter_value().double_value
        self.release_tau_s = self.get_parameter("release_tau_s").get_parameter_value().double_value
        self.jaw_min = self.get_parameter("jaw_min").get_parameter_value().double_value
        self.jaw_max = self.get_parameter("jaw_max").get_parameter_value().double_value
        self.publish_epsilon = self.get_parameter("publish_epsilon").get_parameter_value().double_value

        self._gesture_enabled = self.get_parameter("gesture_enabled").get_parameter_value().bool_value
        self._gesture_suppress_neutral = self.get_parameter("gesture_suppress_neutral").get_parameter_value().bool_value
        self._gesture_min_text_length = self.get_parameter("gesture_min_text_length").get_parameter_value().integer_value

        # ------------------------------------------------------------------ #
        # Microsoft Viseme ID → normalised jaw openness (0..1)
        # ------------------------------------------------------------------ #
        self.ms_viseme_jaw_map = {
            0: 0.0, 1: 0.7, 2: 0.9, 3: 0.8, 4: 0.6, 5: 0.5,
            6: 0.4, 7: 0.3, 8: 0.7, 9: 0.8, 10: 0.7, 11: 0.8,
            12: 0.3, 13: 0.3, 14: 0.4, 15: 0.2, 16: 0.3, 17: 0.2,
            18: 0.1, 19: 0.3, 20: 0.4, 21: 0.0,
        }

        # ------------------------------------------------------------------ #
        # Async / threading state
        # ------------------------------------------------------------------ #
        self._loop = None
        self._stop_event = None
        self._server = None
        self._stopping = threading.Event()

        # Viseme jaw timeline
        self._state_lock = threading.Lock()
        self._active_response_id = None
        self._response_base_time = None
        self._seen_viseme_keys = set()
        self._timeline = []
        self._response_end_s = 0.0

        # Jaw smoother state
        self._current_norm_jaw = 0.0
        self._last_published_value = None

        # Transcript accumulation (for gesture classification)
        self._transcript_lock = threading.Lock()
        self._transcript_parts: list[str] = []
        self._transcript_response_id: str | None = None
        self._gesture_fired_for_response: str | None = None

        self.get_logger().info(f"Upstream S2S server: {self.server_url}")
        self.get_logger().info(
            f"Proxy listening on ws://{self.listen_host}:{self.listen_port}"
        )
        self.get_logger().info(
            f"Jaw smoothing: rate={self.publish_rate_hz}Hz, "
            f"attack={self.attack_tau_s}s, release={self.release_tau_s}s, "
            f"range=[{self.jaw_min}, {self.jaw_max}] rad"
        )
        self.get_logger().info(
            f"Gesture triggering: enabled={self._gesture_enabled}, "
            f"suppress_neutral={self._gesture_suppress_neutral}"
        )

        self.bridge_thread = threading.Thread(
            target=self._run_proxy, name="s2s-websocket-proxy", daemon=True
        )
        self.bridge_thread.start()

    # ======================================================================
    # Proxy entry point
    # ======================================================================

    def _run_proxy(self):
        try:
            asyncio.run(self._proxy_main())
        except Exception as exc:
            if not self._stopping.is_set():
                self.get_logger().error(f"WebSocket proxy stopped: {exc}")

    async def _proxy_main(self):
        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

        control_task = self._loop.create_task(self._jaw_control_loop())

        try:
            async with websocket_serve(
                self._handle_client,
                self.listen_host,
                self.listen_port,
                max_size=None,
                ping_interval=20,
                ping_timeout=20,
            ) as server:
                self._server = server
                self.get_logger().info(
                    f"WebSocket proxy ready at ws://{self.listen_host}:{self.listen_port}"
                )
                await self._stop_event.wait()
        finally:
            control_task.cancel()
            with suppress(asyncio.CancelledError):
                await control_task
            self._reset_timeline_state()
            self._reset_transcript_state()
            self._server = None
            self._loop = None
            self._stop_event = None

    # ======================================================================
    # WebSocket connection handling
    # ======================================================================

    def _upstream_url_for_client(self, client_ws, handler_args):
        client_path = handler_args[0] if handler_args else ""
        if not client_path:
            request = getattr(client_ws, "request", None)
            client_path = getattr(request, "path", "")

        client_query = urlsplit(client_path).query
        upstream = urlsplit(self.server_url)
        query = upstream.query
        if client_query:
            query = f"{query}&{client_query}" if query else client_query

        return urlunsplit(
            (upstream.scheme, upstream.netloc, upstream.path, query, upstream.fragment)
        )

    @staticmethod
    def _forward_headers(websocket):
        headers = getattr(websocket, "request_headers", None)
        if headers is None:
            request = getattr(websocket, "request", None)
            headers = getattr(request, "headers", None)

        if headers is None:
            return []

        excluded = {
            "host", "connection", "upgrade", "content-length",
            "sec-websocket-key", "sec-websocket-version",
            "sec-websocket-extensions", "sec-websocket-protocol",
        }
        return [
            (name, value)
            for name, value in headers.raw_items()
            if name.lower() not in excluded
        ]

    async def _handle_client(self, client_ws, *args):
        self.get_logger().info("S2S talk client connected to proxy")
        upstream_ws = None
        try:
            headers = self._forward_headers(client_ws)
            upstream_url = self._upstream_url_for_client(client_ws, args)
            upstream_ws = await websocket_connect(
                upstream_url,
                extra_headers=headers,
                max_size=None,
                ping_interval=20,
                ping_timeout=20,
            )
            self.get_logger().info("Connected to upstream S2S server")

            client_to_server = asyncio.create_task(
                self._relay(client_ws, upstream_ws, inspect_visemes=False)
            )
            server_to_client = asyncio.create_task(
                self._relay(upstream_ws, client_ws, inspect_visemes=True)
            )
            tasks = {client_to_server, server_to_client}
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            for task in done:
                exception = task.exception()
                if exception is not None:
                    raise exception
        except ConnectionClosed:
            self.get_logger().info("WebSocket connection closed")
        except Exception as exc:
            self.get_logger().error(f"Proxy connection error: {exc}")
        finally:
            self._reset_timeline_state()
            self._reset_transcript_state()
            self.publish_jaw(self.jaw_min, force=True)
            if upstream_ws is not None:
                with suppress(Exception):
                    await upstream_ws.close()
            with suppress(Exception):
                await client_ws.close()
            self.get_logger().info("S2S proxy connection finished")

    async def _relay(self, source, destination, inspect_visemes):
        async for message in source:
            if inspect_visemes and isinstance(message, str):
                data = self._parse_event_json(message)
                if isinstance(data, dict) and self._handle_server_event(data):
                    continue
            await destination.send(message)

    # ======================================================================
    # Server event processing
    # ======================================================================

    @staticmethod
    def _parse_event_json(message):
        clean_msg = message.strip()
        if clean_msg.startswith("EVENT:"):
            clean_msg = clean_msg[len("EVENT:"):].strip()
        try:
            return json.loads(clean_msg)
        except (json.JSONDecodeError, TypeError):
            return None

    def _handle_server_event(self, data: dict) -> bool:
        """Inspect a server→client event.  Return True to consume (not forward).

        Returns False for all events so they are always forwarded to the
        talk client — we only *observe* the stream, never swallow frames.
        """
        event_type = data.get("type", "")

        # --- Viseme events ------------------------------------------------
        if event_type == "speech_to_speech.output_audio.visemes" or "visemes" in data:
            visemes = data.get("visemes")
            response_id = data.get("response_id") or "default"
            if isinstance(visemes, list):
                self._ingest_viseme_batch(response_id, visemes)
            return False  # forward to client

        # --- Transcript streaming (accumulate for gesture classification) --
        if event_type == "response.audio_transcript.delta":
            delta = data.get("delta", "")
            response_id = data.get("response_id", "default")
            if delta:
                self._accumulate_transcript(response_id, delta)
            return False

        # --- Response complete: classify and fire gesture ------------------
        if event_type == "response.done":
            # Extract full transcript from response.done payload (most reliable)
            transcript = self._extract_transcript_from_done(data)
            response_id = (
                data.get("response", {}).get("id")
                or data.get("response_id")
                or "default"
            )
            if transcript:
                # Prefer the done-payload transcript (complete) over accumulated deltas
                with self._transcript_lock:
                    self._transcript_parts = [transcript]
                    self._transcript_response_id = response_id
            self._maybe_fire_gesture(response_id)
            return False

        # --- Speech events: reset state -----------------------------------
        if event_type in (
            "speech.stopped",
            "input_audio_buffer.speech_started",
            "response.cancelled",
        ):
            self.get_logger().debug(f"Event {event_type}; resetting jaw & transcript state")
            self._reset_timeline_state()
            self._reset_transcript_state()

        return False

    @staticmethod
    def _extract_transcript_from_done(data: dict) -> str:
        """Pull the full assistant text from a response.done event."""
        try:
            outputs = data.get("response", {}).get("output", [])
            parts: list[str] = []
            for item in outputs:
                for content in item.get("content", []):
                    t = content.get("transcript", "")
                    if t:
                        parts.append(t)
            return " ".join(parts)
        except Exception:
            return ""

    # ======================================================================
    # Transcript accumulation
    # ======================================================================

    def _accumulate_transcript(self, response_id: str, delta: str) -> None:
        with self._transcript_lock:
            if response_id != self._transcript_response_id:
                self._transcript_parts = []
                self._transcript_response_id = response_id
            self._transcript_parts.append(delta)

            # Fire gesture eagerly once we have enough text (reduces latency)
            full_text = "".join(self._transcript_parts)
            if (
                self._gesture_fired_for_response != response_id
                and len(full_text) >= self._gesture_min_text_length
            ):
                self._classify_and_publish(full_text, response_id, source="delta")

    def _reset_transcript_state(self) -> None:
        with self._transcript_lock:
            self._transcript_parts = []
            self._transcript_response_id = None
            self._gesture_fired_for_response = None

    # ======================================================================
    # Gesture classification and publishing
    # ======================================================================

    def _maybe_fire_gesture(self, response_id: str) -> None:
        """Called on response.done — fire if not already done from deltas."""
        with self._transcript_lock:
            if self._gesture_fired_for_response == response_id:
                return  # already fired from early delta
            full_text = "".join(self._transcript_parts)

        self._classify_and_publish(full_text, response_id, source="done")

    def _classify_and_publish(self, text: str, response_id: str, source: str) -> None:
        """Classify text and publish gesture action if appropriate.

        Must be called with self._transcript_lock held OR be thread-safe on its own.
        Uses _gesture_fired_for_response to ensure one gesture per response.
        """
        if not self._gesture_enabled:
            return

        with self._transcript_lock:
            if self._gesture_fired_for_response == response_id:
                return
            self._gesture_fired_for_response = response_id

        gesture, matched = classify_with_confidence(text)

        if self._gesture_suppress_neutral and gesture == "neutral":
            self.get_logger().debug(
                f"[gesture/{source}] Suppressed neutral for response_id={response_id}"
            )
            return

        self.get_logger().info(
            f"[gesture/{source}] '{gesture}' (matched: '{matched}') "
            f"for response_id={response_id}, text_len={len(text)}"
        )
        self._publish_gesture(gesture)

    def _publish_gesture(self, gesture: str) -> None:
        msg = String()
        msg.data = json.dumps({"action": gesture})
        self.action_pub.publish(msg)
        self.get_logger().debug(f"Published /robot/action: {msg.data}")

    # ======================================================================
    # Viseme jaw timeline (unchanged from original)
    # ======================================================================

    def _ingest_viseme_batch(self, response_id, visemes):
        if self._loop is None or not visemes:
            return
        now = self._loop.time()
        with self._state_lock:
            if response_id != self._active_response_id:
                self._active_response_id = response_id
                self._response_base_time = now
                self._seen_viseme_keys.clear()
                self._timeline.clear()
                self._response_end_s = 0.0
                self.get_logger().debug(f"Started viseme timeline for response_id={response_id}")

            added = 0
            for item in visemes:
                viseme_id = int(item.get("viseme", 0))
                start_s = float(item.get("start_s", 0.0))
                end_s = float(item.get("end_s", start_s))
                entry_key = (round(start_s, 4), round(end_s, 4), viseme_id)
                if entry_key in self._seen_viseme_keys:
                    continue
                self._seen_viseme_keys.add(entry_key)
                norm_jaw = self.ms_viseme_jaw_map.get(viseme_id, 0.0)
                self._timeline.append((start_s, end_s, viseme_id, norm_jaw))
                self._response_end_s = max(self._response_end_s, end_s)
                added += 1

            if added:
                self._timeline.sort(key=lambda entry: entry[0])
                self.get_logger().debug(
                    f"Queued {added} new visemes (total={len(self._timeline)}, "
                    f"end={self._response_end_s:.3f}s)"
                )

    def _reset_timeline_state(self):
        with self._state_lock:
            self._active_response_id = None
            self._response_base_time = None
            self._seen_viseme_keys.clear()
            self._timeline.clear()
            self._response_end_s = 0.0

    def _sample_target_norm_jaw(self, now):
        with self._state_lock:
            if self._response_base_time is None or not self._timeline:
                return 0.0
            elapsed = now - self._response_base_time
            if elapsed < 0.0:
                return 0.0
            if elapsed > self._response_end_s:
                self._timeline.clear()
                return 0.0

            target = 0.0
            active_end = 0.0
            for start_s, end_s, _, norm_jaw in self._timeline:
                if start_s <= elapsed:
                    target = norm_jaw
                    active_end = end_s
                else:
                    break
            if active_end > 0.0 and elapsed > active_end:
                return 0.0
            return target

    async def _jaw_control_loop(self):
        loop = asyncio.get_running_loop()
        rate_hz = max(10.0, float(self.publish_rate_hz))
        dt = 1.0 / rate_hz
        try:
            while True:
                tick_start = loop.time()
                target_norm = self._sample_target_norm_jaw(tick_start)
                tau = self.attack_tau_s if target_norm > self._current_norm_jaw else self.release_tau_s
                alpha = 1.0 if tau <= 0.0 else 1.0 - pow(2.718281828459045, -dt / tau)
                self._current_norm_jaw += alpha * (target_norm - self._current_norm_jaw)
                if target_norm == 0.0 and self._current_norm_jaw < 0.005:
                    self._current_norm_jaw = 0.0
                scaled_jaw = self.jaw_min + self._current_norm_jaw * (self.jaw_max - self.jaw_min)
                self.publish_jaw(scaled_jaw)
                sleep_time = dt - (loop.time() - tick_start)
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
        except asyncio.CancelledError:
            pass

    def publish_jaw(self, value, force=False):
        val = float(value)
        if (
            not force
            and self._last_published_value is not None
            and abs(val - self._last_published_value) < self.publish_epsilon
        ):
            return
        self._last_published_value = val
        msg = Float64()
        msg.data = val
        self.jaw_pub.publish(msg)
        self.get_logger().debug(f"Published /jaw: {msg.data:.4f} rad")

    # ======================================================================
    # Cleanup
    # ======================================================================

    def destroy_node(self):
        self._stopping.set()
        if self._loop is not None and self._stop_event is not None:
            with suppress(RuntimeError):
                self._loop.call_soon_threadsafe(self._stop_event.set)
        if hasattr(self, "bridge_thread") and self.bridge_thread.is_alive():
            self.bridge_thread.join(timeout=3.0)
        self.publish_jaw(self.jaw_min, force=True)
        return super().destroy_node()


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


if __name__ == "__main__":
    main()

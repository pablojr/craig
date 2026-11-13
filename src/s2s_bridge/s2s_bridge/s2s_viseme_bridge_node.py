#!/usr/bin/env python3
"""ROS 2 WebSocket proxy for S2S viseme interception.

The S2S talk client connects to this node. The node opens one upstream
connection to the S2S server, consumes viseme events to publish /jaw, and
forwards every other WebSocket frame unchanged in both directions.
"""

import asyncio
import json
import threading
from contextlib import suppress
from urllib.parse import urlsplit, urlunsplit

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

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


class SpeechToSpeechVisemeBridge(Node):
    def __init__(self):
        super().__init__("s2s_viseme_bridge_node")

        self.jaw_pub = self.create_publisher(Float32, "/jaw", 10)

        # server_url is the UPSTREAM S2S server,
        # not the URL used by the talk client.
        # Point the talk client to ws://<listen_host>:<listen_port>.
        self.declare_parameter(
            "server_url", "ws://192.168.0.202:8765/v1/realtime"
        )
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 18765)

        self.server_url = (
            self.get_parameter("server_url").get_parameter_value().string_value
        )
        self.listen_host = (
            self.get_parameter("listen_host").get_parameter_value().string_value
        )
        self.listen_port = (
            self.get_parameter("listen_port").get_parameter_value().integer_value
        )

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
        self._loop = None
        self._stop_event = None
        self._server = None
        self._stopping = threading.Event()
        self._playback_tasks = set()

        self.get_logger().info(f"Upstream S2S server: {self.server_url}")
        self.get_logger().info(
            f"Proxy listening on ws://{self.listen_host}:{self.listen_port}"
        )

        self.bridge_thread = threading.Thread(
            target=self._run_proxy, name="s2s-websocket-proxy", daemon=True
        )
        self.bridge_thread.start()

    def _run_proxy(self):
        try:
            asyncio.run(self._proxy_main())
        except Exception as exc:
            if not self._stopping.is_set():
                self.get_logger().error(f"WebSocket proxy stopped: {exc}")

    async def _proxy_main(self):
        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()

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
                    f"WebSocket proxy ready at "
                    f"ws://{self.listen_host}:{self.listen_port}"
                )
                await self._stop_event.wait()
        finally:
            await self._cancel_viseme_playback(reset_jaw=False)
            self._server = None
            self._loop = None
            self._stop_event = None

    def _upstream_url_for_client(self, client_ws, handler_args):
        """Keep client query parameters (for example, model=...) upstream."""
        client_path = handler_args[0] if handler_args else ""
        if not client_path:
            request = getattr(client_ws, "request", None)
            client_path = getattr(request, "path", "")

        client_query = urlsplit(client_path).query
        upstream = urlsplit(self.server_url)

        # The configured upstream path remains authoritative; only the
        # client's query string is appended to it.
        query = upstream.query
        if client_query:
            query = f"{query}&{client_query}" if query else client_query

        return urlunsplit(
            (upstream.scheme, upstream.netloc, upstream.path, query, upstream.fragment)
        )

    @staticmethod
    def _forward_headers(websocket):
        """Copy client application headers to the upstream handshake.

        WebSocket handshake/framing headers are generated independently for
        each connection and must not be copied.
        """
        headers = getattr(websocket, "request_headers", None)
        if headers is None:
            request = getattr(websocket, "request", None)
            headers = getattr(request, "headers", None)

        if headers is None:
            return []

        excluded = {
            "host",
            "connection",
            "upgrade",
            "content-length",
            "sec-websocket-key",
            "sec-websocket-version",
            "sec-websocket-extensions",
            "sec-websocket-protocol",
        }

        return [
            (name, value)
            for name, value in headers.raw_items()
            if name.lower() not in excluded
        ]

    async def _handle_client(self, client_ws, *args):
        """Handle one talk-client connection with one upstream S2S session."""
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

            # Relay both directions concurrently. Only server-to-client
            # messages are inspected for viseme events.
            client_to_server = asyncio.create_task(
                self._relay(client_ws, upstream_ws, inspect_visemes=False)
            )
            server_to_client = asyncio.create_task(
                self._relay(upstream_ws, client_ws, inspect_visemes=True)
            )

            tasks = {client_to_server, server_to_client}
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_COMPLETED
            )

            for task in pending:
                task.cancel()

            await asyncio.gather(*pending, return_exceptions=True)

            # Surface unexpected relay errors for logging.
            for task in done:
                exception = task.exception()
                if exception is not None:
                    raise exception

        except ConnectionClosed:
            self.get_logger().info("WebSocket connection closed")
        except Exception as exc:
            self.get_logger().error(f"Proxy connection error: {exc}")
        finally:
            await self._cancel_viseme_playback(reset_jaw=True)

            if upstream_ws is not None:
                with suppress(Exception):
                    await upstream_ws.close()

            with suppress(Exception):
                await client_ws.close()

            self.get_logger().info("S2S proxy connection finished")

    async def _relay(self, source, destination, inspect_visemes):
        """Forward frames unchanged, optionally consuming viseme events."""
        async for message in source:
            if inspect_visemes and isinstance(message, str):
                data = self._parse_event_json(message)
                if isinstance(data, dict) and self._handle_server_event(data):
                    # Viseme events are consumed by the proxy. All other
                    # messages are forwarded byte-for-byte/text-for-text.
                    continue

            await destination.send(message)

    @staticmethod
    def _parse_event_json(message):
        """Parse raw JSON string, stripping optional 'EVENT:' prefix if present."""
        clean_msg = message.strip()
        if clean_msg.startswith("EVENT:"):
            clean_msg = clean_msg[len("EVENT:"):].strip()
        try:
            return json.loads(clean_msg)
        except (json.JSONDecodeError, TypeError):
            return None

    def _handle_server_event(self, data):
        """Return True when an event was intercepted and consumed as a viseme event."""
        event_type = data.get("type")

        if event_type == "speech_to_speech.output_audio.visemes" or "visemes" in data:
            visemes = data.get("visemes")
            if isinstance(visemes, list):
                self.get_logger().debug(
                    f"Intercepted viseme batch ({len(visemes)} items)"
                )
                self._schedule_visemes(visemes)
            return True

        # Cancel active jaw motion and close mouth on interruption or speech stop
        if event_type in (
            "speech.stopped",
            "input_audio_buffer.speech_started",
            "response.cancelled",
        ):
            self.get_logger().debug(f"Event {event_type}; resetting jaw")
            self._schedule_cancel_and_reset()

        return False

    def _schedule_visemes(self, visemes):
        """Schedule timed viseme playback without blocking the WebSocket relay."""
        if self._loop is None or not visemes:
            return

        # Cancel any overlapping playback from an earlier response/chunk
        for task in list(self._playback_tasks):
            task.cancel()

        task = self._loop.create_task(self._play_viseme_sequence(visemes))
        self._playback_tasks.add(task)
        task.add_done_callback(self._playback_tasks.discard)

    def _schedule_cancel_and_reset(self):
        """Cancel active viseme tasks from synchronous event handler and close jaw."""
        for task in list(self._playback_tasks):
            task.cancel()
        self.publish_jaw(0.0)

    async def _cancel_viseme_playback(self, reset_jaw=True):
        """Await cancellation of all active viseme playback tasks."""
        tasks = list(self._playback_tasks)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        if reset_jaw:
            self.publish_jaw(0.0)

    async def _play_viseme_sequence(self, visemes):
        """Publish jaw positions according to each viseme's start_s and end_s."""
        loop = asyncio.get_running_loop()
        base_time = loop.time()

        try:
            # Sort chronologically in case entries arrive out of order
            ordered = sorted(visemes, key=lambda item: float(item.get("start_s", 0.0)))

            for item in ordered:
                viseme_id = int(item.get("viseme", 0))
                start_s = float(item.get("start_s", 0.0))

                delay = (base_time + start_s) - loop.time()
                if delay > 0:
                    await asyncio.sleep(delay)

                jaw_val = self.ms_viseme_jaw_map.get(viseme_id)
                if jaw_val is not None:
                    self.publish_jaw(jaw_val)
                else:
                    self.get_logger().warning(
                        f"Viseme {viseme_id!r} has no jaw mapping"
                    )

            # Close the jaw at the end_s timestamp of the final viseme
            last_end_s = float(ordered[-1].get("end_s", 0.0))
            end_delay = (base_time + last_end_s) - loop.time()
            if end_delay > 0:
                await asyncio.sleep(end_delay)

            self.publish_jaw(0.0)
        except asyncio.CancelledError:
            pass

    def publish_jaw(self, value):
        msg = Float32()
        msg.data = float(value)
        self.jaw_pub.publish(msg)
        self.get_logger().debug(f"Published /jaw: {msg.data}")

    def destroy_node(self):
        self._stopping.set()

        # Stop the asyncio server cleanly from the ROS executor thread.
        if self._loop is not None and self._stop_event is not None:
            with suppress(RuntimeError):
                self._loop.call_soon_threadsafe(self._stop_event.set)

        if (
            hasattr(self, "bridge_thread")
            and self.bridge_thread.is_alive()
        ):
            self.bridge_thread.join(timeout=3.0)

        self.publish_jaw(0.0)
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

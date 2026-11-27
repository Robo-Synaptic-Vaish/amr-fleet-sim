#!/usr/bin/env python3
"""
AMR Fleet Live Dashboard & Web Bridge Node (ROS 2 Jazzy)
--------------------------------------------------------
Provides a high-performance, real-time web bridge and dashboard server
for monitoring two-AMR decentralized coordination in Gazebo simulation.

Architecture:
  - Subscribes to /fleet/robot_states, /fleet/emergency_stop, and robot odometry/cmd_vel.
  - Computes real-time inter-robot separation, collision risk, and coordination status.
  - Automatically generates real-time event logs on safety & conflict transitions.
  - Serves an embedded modern dark-themed web dashboard on http://localhost:8080.
  - Streams continuous telemetry to web browsers via Server-Sent Events (SSE) at 10 Hz.
  - Exposes REST control endpoint to trigger/clear physical /fleet/emergency_stop.
"""

import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from amr_fleet_msgs.msg import RobotState

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


class FleetDashboardNode(Node):
    """ROS 2 node integrating live fleet state with an HTTP/SSE web bridge."""

    def __init__(self):
        super().__init__('fleet_dashboard_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('stale_timeout', 1.5)

        self.server_port = int(self.get_parameter('port').value)
        self.server_host = str(self.get_parameter('host').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)

        # -------------------------------------------------------------
        # Live State Storage (Thread-Safe)
        # -------------------------------------------------------------
        self.lock = threading.Lock()

        # Telemetry for both AMRs
        self.robots = {
            'amr': {
                'id': 'AMR 01',
                'name': 'amr',
                'priority': 1,
                'x': -2.0,
                'y': 0.8,
                'heading': 0.0,
                'lin_vel': 0.0,
                'ang_vel': 0.0,
                'status': 'WAITING',
                'decision': 'PROCEED',
                'gate_action': 'ALLOWED',
                'risk': 'LOW',
                'current_task': 'UNASSIGNED',
                'online': False,
                'stale': False,
                'last_time': None,
                'age_ms': 0
            },
            'amr2': {
                'id': 'AMR 02',
                'name': 'amr2',
                'priority': 2,
                'x': -2.0,
                'y': -0.8,
                'heading': 0.0,
                'lin_vel': 0.0,
                'ang_vel': 0.0,
                'status': 'WAITING',
                'decision': 'PROCEED',
                'gate_action': 'ALLOWED',
                'risk': 'LOW',
                'current_task': 'UNASSIGNED',
                'online': False,
                'stale': False,
                'last_time': None,
                'age_ms': 0
            }
        }

        # System Metrics
        self.distance: Optional[float] = 1.60
        self.collision_risk = "SAFE"
        self.coordination_state = "NORMAL NAVIGATION"
        self.coordination_message = "All robots operating normally"
        self.safety_intervention = "ALLOWED"
        self.emergency_stop = False
        self.in_conflict = False

        # Event Feed History (up to 60 events)
        self.events: List[Dict] = []
        self._add_event("SYSTEM", "info", "Fleet Dashboard initialized and listening for ROS 2 telemetry")

        # -------------------------------------------------------------
        # ROS 2 Subscriptions & Publishers
        # -------------------------------------------------------------
        self.state_sub = self.create_subscription(
            RobotState, '/fleet/robot_states', self.fleet_state_callback, 20
        )

        self.estop_sub = self.create_subscription(
            Bool, '/fleet/emergency_stop', self.estop_callback, 10
        )
        self.estop_pub = self.create_publisher(
            Bool, '/fleet/emergency_stop', 10
        )

        # Dedicated Safety Status Subscriptions (direct from fleet_safety_node)
        self.safety_sub_amr = self.create_subscription(
            String, '/model/amr/safety_status', lambda msg: self.safety_status_callback('amr', msg), 10
        )
        self.safety_sub_amr2 = self.create_subscription(
            String, '/model/amr2/safety_status', lambda msg: self.safety_status_callback('amr2', msg), 10
        )

        # Background 10 Hz calculation timer
        self.timer = self.create_timer(0.1, self.update_fleet_metrics)

        # Locate Web UI HTML file
        self.html_file_path = self._find_html_file()
        self.get_logger().info(f"Dashboard HTML source: {self.html_file_path}")

        # Start Web Server in background thread
        self.http_server = None
        self.server_thread = threading.Thread(target=self._run_web_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"AMR Fleet Live Dashboard running at http://localhost:{self.server_port}"
        )

    def _find_html_file(self) -> str:
        """Locates index.html in workspace source or install directories."""
        candidates = [
            os.path.join(os.path.dirname(__file__), '..', 'web', 'index.html'),
            os.path.join(os.path.dirname(__file__), 'web', 'index.html'),
            '/mnt/c/Users/reena/amr-fleet-sim/src/amr_description/web/index.html',
        ]
        if get_package_share_directory:
            try:
                candidates.insert(0, os.path.join(get_package_share_directory('amr_description'), 'web', 'index.html'))
            except Exception:
                pass

        for p in candidates:
            abs_p = os.path.abspath(p)
            if os.path.exists(abs_p):
                return abs_p
        return ""

    def _add_event(self, event_type: str, level: str, message: str, robot_id: str = "") -> None:
        """Appends a new structured event to the live history."""
        curr_time = time.strftime("%H:%M:%S")
        self.events.append({
            'id': len(self.events) + 1,
            'time': curr_time,
            'type': event_type,
            'level': level,
            'message': message,
            'robot_id': robot_id
        })
        if len(self.events) > 60:
            self.events.pop(0)

    # -------------------------------------------------------------
    # ROS Callbacks
    # -------------------------------------------------------------
    def fleet_state_callback(self, msg: RobotState) -> None:
        now = time.monotonic()
        name = msg.robot_id
        if name not in self.robots:
            return

        with self.lock:
            bot = self.robots[name]
            was_online = bot['online']

            bot['x'] = float(msg.position_x)
            bot['y'] = float(msg.position_y)
            bot['heading'] = float(msg.heading)
            bot['lin_vel'] = float(msg.linear_velocity)
            bot['ang_vel'] = float(msg.angular_velocity)
            bot['status'] = str(msg.status)
            bot['current_task'] = str(msg.current_task)
            bot['last_time'] = now
            bot['online'] = True
            bot['stale'] = False
            bot['age_ms'] = 0

            # Log connection event
            if not was_online:
                self._add_event("CONNECT", "success", f"Connected to {bot['id']} ({name}) state stream", name)

    def safety_status_callback(self, robot_id: str, msg: String) -> None:
        """Processes actual safety gate decision and risk broadcast from fleet_safety_node."""
        try:
            data = json.loads(msg.data)
            with self.lock:
                if robot_id in self.robots:
                    bot = self.robots[robot_id]
                    old_decision = bot.get('decision', '')
                    new_decision = str(data.get('decision', 'PROCEED'))
                    new_gate = str(data.get('gate_action', 'ALLOWED'))
                    new_risk = str(data.get('risk', 'LOW'))

                    bot['decision'] = new_decision
                    bot['gate_action'] = new_gate
                    bot['risk'] = new_risk

                    # Log transition if safety decision changed
                    if old_decision and old_decision != new_decision:
                        level = "critical" if new_decision in ("EMERGENCY_STOP", "YIELD") else "warn" if new_decision in ("SLOW", "COMM_LOSS_SAFE") else "info"
                        self._add_event("SAFETY", level, f"{bot['id']} safety decision -> {new_decision} ({new_gate})", robot_id)
        except Exception as ex:
            self.get_logger().error(f"Error parsing safety_status for {robot_id}: {ex}")

    def estop_callback(self, msg: Bool) -> None:
        with self.lock:
            if msg.data != self.emergency_stop:
                self.emergency_stop = msg.data
                level = "critical" if self.emergency_stop else "success"
                status_str = "ACTIVATED" if self.emergency_stop else "CLEARED"
                self._add_event("E-STOP", level, f"Emergency Stop {status_str} across fleet")

    # -------------------------------------------------------------
    # Periodic Metrics Calculation (10 Hz)
    # -------------------------------------------------------------
    def update_fleet_metrics(self) -> None:
        now = time.monotonic()
        with self.lock:
            # 1. Update online/stale status
            for name, bot in self.robots.items():
                if bot['last_time'] is None:
                    bot['online'] = False
                    bot['stale'] = False
                    bot['age_ms'] = 0
                else:
                    age = now - bot['last_time']
                    bot['age_ms'] = int(age * 1000)
                    if age > self.stale_timeout:
                        if bot['online']:
                            bot['online'] = False
                            bot['stale'] = True
                            self._add_event("TIMEOUT", "warn", f"{bot['id']} communication timeout (>1.5s)", name)
                    else:
                        bot['online'] = True
                        bot['stale'] = False

            bot1 = self.robots['amr']
            bot2 = self.robots['amr2']

            # 2. Inter-Robot Distance
            if bot1['last_time'] is not None and bot2['last_time'] is not None:
                dx = bot2['x'] - bot1['x']
                dy = bot2['y'] - bot1['y']
                self.distance = math.sqrt(dx * dx + dy * dy)
            else:
                self.distance = None

            # 3. Fleet Coordination State (Aggregated directly from node safety statuses)
            dec1 = bot1.get('decision', 'PROCEED')
            dec2 = bot2.get('decision', 'PROCEED')
            gate1 = bot1.get('gate_action', 'ALLOWED')
            gate2 = bot2.get('gate_action', 'ALLOWED')

            if self.emergency_stop or dec1 == 'EMERGENCY_STOP' or dec2 == 'EMERGENCY_STOP':
                self.in_conflict = False
                self.collision_risk = "CRITICAL"
                self.coordination_state = "EMERGENCY_STOP"
                self.coordination_message = "SAFETY OVERRIDE: Emergency Stop Active"
                self.safety_intervention = "FULL HALT"

            elif (not bot1['online']) or (not bot2['online']):
                self.in_conflict = False
                if bot1['stale'] or bot2['stale']:
                    self.collision_risk = "WARNING"
                    self.coordination_state = "COMM_LOSS"
                    self.coordination_message = "Communication lost with peer - safe crawl active"
                    self.safety_intervention = "LIMITED"
                else:
                    self.collision_risk = "SAFE"
                    self.coordination_state = "DISCOVERY"
                    self.coordination_message = "Waiting for both AMRs to connect"
                    self.safety_intervention = "ALLOWED"

            elif dec1 == 'YIELD' or dec2 == 'YIELD':
                self.in_conflict = True
                self.collision_risk = "CRITICAL"
                self.coordination_state = "CONFLICT DETECTED"
                if dec2 == 'YIELD':
                    self.coordination_message = "AMR 01 HAS PRIORITY - AMR 02 YIELDING"
                    self.safety_intervention = f"AMR 02 FORWARD {gate2}"
                else:
                    self.coordination_message = "AMR 02 HAS PRIORITY - AMR 01 YIELDING"
                    self.safety_intervention = f"AMR 01 FORWARD {gate1}"

            elif dec1 == 'COMM_LOSS_SAFE' or dec2 == 'COMM_LOSS_SAFE':
                self.in_conflict = False
                self.collision_risk = "WARNING"
                self.coordination_state = "COMM_LOSS"
                self.coordination_message = "Communication timeout - safe crawl active"
                self.safety_intervention = "LIMITED"

            elif dec1 == 'SLOW' or dec2 == 'SLOW':
                self.in_conflict = False
                self.collision_risk = "WARNING"
                self.coordination_state = "CAUTION ZONE"
                self.coordination_message = "Caution proximity: Speed scaled to 50%"
                self.safety_intervention = "SCALED 50%"

            else:
                self.in_conflict = False
                self.collision_risk = "SAFE"
                self.coordination_state = "NORMAL"
                self.coordination_message = "Clear aisle navigation"
                self.safety_intervention = "ALLOWED"

    def get_snapshot_json(self) -> str:
        """Returns JSON representation of all current dashboard data."""
        with self.lock:
            payload = {
                'amr1': self.robots['amr'],
                'amr2': self.robots['amr2'],
                'distance': self.distance,
                'risk': self.collision_risk,
                'coordination_state': self.coordination_state,
                'coordination_message': self.coordination_message,
                'safety_intervention': self.safety_intervention,
                'emergency_stop': self.emergency_stop,
                'events': self.events
            }
            return json.dumps(payload)

    # -------------------------------------------------------------
    # Embedded HTTP + SSE Web Server
    # -------------------------------------------------------------
    def _run_web_server(self):
        node_ref = self

        class DashboardHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                # Suppress noisy HTTP access logs in console
                pass

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/' or parsed.path == '/index.html':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    if node_ref.html_file_path and os.path.exists(node_ref.html_file_path):
                        with open(node_ref.html_file_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.wfile.write(b"<h1>Dashboard HTML not found</h1>")

                elif parsed.path == '/api/state':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(node_ref.get_snapshot_json().encode('utf-8'))

                elif parsed.path == '/stream':
                    # Server-Sent Events (SSE) stream
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    try:
                        while rclpy.ok():
                            payload = node_ref.get_snapshot_json()
                            msg = f"data: {payload}\n\n"
                            self.wfile.write(msg.encode('utf-8'))
                            self.wfile.flush()
                            time.sleep(0.1)  # 10 Hz live update rate
                    except (BrokenPipeError, ConnectionResetError):
                        pass

                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path == '/api/estop':
                    content_length = int(self.headers.get('Content-Length', 0))
                    body = self.rfile.read(content_length).decode('utf-8')
                    try:
                        req_data = json.loads(body)
                        target_active = bool(req_data.get('active', True))
                        # Publish to ROS 2 /fleet/emergency_stop topic
                        msg = Bool(data=target_active)
                        for _ in range(3):
                            node_ref.estop_pub.publish(msg)
                            time.sleep(0.01)

                        with node_ref.lock:
                            node_ref.emergency_stop = target_active

                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps({'status': 'ok', 'emergency_stop': target_active}).encode('utf-8'))
                    except Exception as ex:
                        self.send_response(400)
                        self.end_headers()
                        self.wfile.write(json.dumps({'error': str(ex)}).encode('utf-8'))
                else:
                    self.send_response(404)
                    self.end_headers()

        self.http_server = ThreadingHTTPServer((self.server_host, self.server_port), DashboardHandler)
        self.http_server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = FleetDashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.http_server:
            node.http_server.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()

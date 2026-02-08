"""
SCARA Robot Dashboard - Communication Client
TCP client for communicating with the C simulation server
"""

import socket
import json
import threading
import time
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class Telemetry:
    """Telemetry data from simulation"""
    sequence: int = 0
    timestamp: float = 0.0
    joints: Dict[str, float] = None
    pose: Dict[str, float] = None
    errors: Dict[str, float] = None
    mode: str = "idle"
    ik_valid: bool = True
    at_target: bool = True
    progress: float = 0.0
    
    def __post_init__(self):
        if self.joints is None:
            self.joints = {"theta1": 0, "theta2": 0, "d3": 0, "theta4": 0}
        if self.pose is None:
            self.pose = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        if self.errors is None:
            self.errors = {"theta1": 0, "theta2": 0, "d3": 0, "theta4": 0}


class CommClient:
    """TCP client for communicating with SCARA simulation server"""
    
    def __init__(self, host: str = "localhost", port: int = 5555):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        
        self.recv_thread: Optional[threading.Thread] = None
        self.recv_buffer = ""
        
        self.latest_telemetry = Telemetry()
        self.telemetry_callback: Optional[Callable[[Telemetry], None]] = None
        
        self.lock = threading.Lock()
        self.command_sequence = 0
    
    def connect(self) -> bool:
        """Connect to the simulation server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.1)  # Non-blocking for recv
            
            self.connected = True
            self.running = True
            
            # Start receive thread
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()
            
            print(f"Connected to {self.host}:{self.port}")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from server"""
        self.running = False
        self.connected = False
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
    
    def _recv_loop(self):
        """Background thread for receiving telemetry"""
        while self.running and self.connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    self.connected = False
                    break
                
                self.recv_buffer += data.decode('utf-8')
                
                # Process complete messages (newline delimited)
                while '\n' in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line.strip():
                        self._process_message(line.strip())
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                    self.connected = False
                break
    
    def _process_message(self, message: str):
        """Process received JSON message"""
        try:
            data = json.loads(message)
            
            with self.lock:
                self.latest_telemetry = Telemetry(
                    sequence=data.get("seq", 0),
                    timestamp=data.get("timestamp", 0.0),
                    joints=data.get("joints", {}),
                    pose=data.get("pose", {}),
                    errors=data.get("errors", {}),
                    mode=data.get("mode", "idle"),
                    ik_valid=data.get("ik_valid", True),
                    at_target=data.get("at_target", True),
                    progress=data.get("progress", 0.0)
                )
            
            if self.telemetry_callback:
                self.telemetry_callback(self.latest_telemetry)
                
        except json.JSONDecodeError as e:
            print(f"JSON parse error: {e}")
    
    def get_telemetry(self) -> Telemetry:
        """Get latest telemetry (thread-safe)"""
        with self.lock:
            return self.latest_telemetry
    
    def send_goto_pose(self, x: float, y: float, z: float, yaw: float) -> bool:
        """Send goto pose command"""
        cmd = {
            "seq": self._next_sequence(),
            "timestamp": time.time(),
            "command": "goto",
            "target": {"x": x, "y": y, "z": z, "yaw": yaw}
        }
        return self._send_command(cmd)
    
    def send_goto_joints(self, theta1: float, theta2: float, d3: float, theta4: float) -> bool:
        """Send goto joints command"""
        cmd = {
            "seq": self._next_sequence(),
            "timestamp": time.time(),
            "command": "goto_joints",
            "target": {"theta1": theta1, "theta2": theta2, "d3": d3, "theta4": theta4}
        }
        return self._send_command(cmd)
    
    def send_stop(self) -> bool:
        """Send stop command"""
        cmd = {
            "seq": self._next_sequence(),
            "timestamp": time.time(),
            "command": "stop"
        }
        return self._send_command(cmd)
    
    def send_reset(self) -> bool:
        """Send reset command"""
        cmd = {
            "seq": self._next_sequence(),
            "timestamp": time.time(),
            "command": "reset"
        }
        return self._send_command(cmd)
    
    def _next_sequence(self) -> int:
        """Get next command sequence number"""
        self.command_sequence += 1
        return self.command_sequence
    
    def _send_command(self, cmd: Dict[str, Any]) -> bool:
        """Send JSON command to server"""
        if not self.connected or not self.socket:
            return False
        
        try:
            message = json.dumps(cmd) + '\n'
            self.socket.sendall(message.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False
            return False
    
    def set_telemetry_callback(self, callback: Callable[[Telemetry], None]):
        """Set callback for telemetry updates"""
        self.telemetry_callback = callback


# Test the client
if __name__ == "__main__":
    client = CommClient()
    
    if client.connect():
        print("Connected! Receiving telemetry...")
        
        def on_telemetry(tel: Telemetry):
            print(f"Pose: x={tel.pose['x']:.3f}, y={tel.pose['y']:.3f}, z={tel.pose['z']:.3f}")
        
        client.set_telemetry_callback(on_telemetry)
        
        try:
            time.sleep(5)
            print("Sending goto command...")
            client.send_goto_pose(0.35, 0.2, 0.35, 0.5)
            time.sleep(10)
        except KeyboardInterrupt:
            pass
        
        client.disconnect()
    else:
        print("Connection failed")

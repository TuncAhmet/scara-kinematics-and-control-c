"""
SCARA Robot Dashboard - 2D Visualization
Renders the robot in top-view (XY plane) using Pygame
"""

import pygame
import math
import numpy as np
from typing import Tuple, Optional, Dict

# Colors
COLOR_BACKGROUND = (20, 25, 30)
COLOR_GRID = (40, 45, 50)
COLOR_WORKSPACE = (30, 60, 80)
COLOR_LINK1 = (70, 130, 180)       # Steel blue
COLOR_LINK2 = (100, 149, 237)      # Cornflower blue
COLOR_JOINT = (255, 200, 100)      # Gold
COLOR_END_EFFECTOR = (50, 205, 50) # Lime green
COLOR_TARGET = (255, 99, 71)       # Tomato
COLOR_TEXT = (200, 200, 200)
COLOR_AXIS = (80, 80, 80)


class SCARAVisualization:
    """2D visualization of SCARA robot (top-view XY plane)"""
    
    def __init__(self, surface: pygame.Surface, 
                 center: Tuple[int, int],
                 scale: float = 400.0,
                 L1: float = 0.30,
                 L2: float = 0.25):
        self.surface = surface
        self.center = center
        self.scale = scale  # Pixels per meter
        self.L1 = L1
        self.L2 = L2
        
        # Robot state
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.d3 = 0.0
        self.theta4 = 0.0
        
        # Target
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        
        # Fonts
        pygame.font.init()
        self.font_small = pygame.font.SysFont('Consolas', 12)
        self.font_medium = pygame.font.SysFont('Consolas', 16)
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (meters) to screen coordinates (pixels)"""
        sx = int(self.center[0] + x * self.scale)
        sy = int(self.center[1] - y * self.scale)  # Flip Y axis
        return (sx, sy)
    
    def update_state(self, theta1: float, theta2: float, d3: float, theta4: float):
        """Update robot joint state"""
        self.theta1 = theta1
        self.theta2 = theta2
        self.d3 = d3
        self.theta4 = theta4
    
    def set_target(self, x: Optional[float], y: Optional[float]):
        """Set target position (None to clear)"""
        self.target_x = x
        self.target_y = y
    
    def draw(self):
        """Draw the robot visualization"""
        self._draw_workspace()
        self._draw_grid()
        self._draw_axes()
        self._draw_robot()
        if self.target_x is not None and self.target_y is not None:
            self._draw_target()
    
    def _draw_workspace(self):
        """Draw workspace boundaries"""
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        # Outer circle (maximum reach)
        pygame.draw.circle(self.surface, COLOR_WORKSPACE, self.center,
                          int(max_reach * self.scale), 2)
        
        # Inner circle (minimum reach)
        pygame.draw.circle(self.surface, COLOR_WORKSPACE, self.center,
                          int(min_reach * self.scale), 1)
    
    def _draw_grid(self):
        """Draw background grid"""
        grid_spacing = 0.1  # 10cm
        grid_range = 0.7
        
        for i in np.arange(-grid_range, grid_range + grid_spacing, grid_spacing):
            # Vertical lines
            p1 = self.world_to_screen(i, -grid_range)
            p2 = self.world_to_screen(i, grid_range)
            pygame.draw.line(self.surface, COLOR_GRID, p1, p2, 1)
            
            # Horizontal lines
            p1 = self.world_to_screen(-grid_range, i)
            p2 = self.world_to_screen(grid_range, i)
            pygame.draw.line(self.surface, COLOR_GRID, p1, p2, 1)
    
    def _draw_axes(self):
        """Draw coordinate axes"""
        axis_length = 0.6
        
        # X axis (red)
        p1 = self.world_to_screen(0, 0)
        p2 = self.world_to_screen(axis_length, 0)
        pygame.draw.line(self.surface, (150, 50, 50), p1, p2, 2)
        label = self.font_small.render("X", True, (150, 50, 50))
        self.surface.blit(label, (p2[0] + 5, p2[1] - 5))
        
        # Y axis (green)
        p2 = self.world_to_screen(0, axis_length)
        pygame.draw.line(self.surface, (50, 150, 50), p1, p2, 2)
        label = self.font_small.render("Y", True, (50, 150, 50))
        self.surface.blit(label, (p2[0] + 5, p2[1] - 15))
    
    def _draw_robot(self):
        """Draw the robot links and joints"""
        # Compute joint positions using FK
        # Base at origin
        base = (0.0, 0.0)
        
        # Elbow position
        elbow_x = self.L1 * math.cos(self.theta1)
        elbow_y = self.L1 * math.sin(self.theta1)
        elbow = (elbow_x, elbow_y)
        
        # End-effector position
        theta12 = self.theta1 + self.theta2
        ee_x = elbow_x + self.L2 * math.cos(theta12)
        ee_y = elbow_y + self.L2 * math.sin(theta12)
        end_effector = (ee_x, ee_y)
        
        # Convert to screen coordinates
        base_screen = self.world_to_screen(*base)
        elbow_screen = self.world_to_screen(*elbow)
        ee_screen = self.world_to_screen(*end_effector)
        
        # Draw links with gradient effect
        self._draw_link(base_screen, elbow_screen, COLOR_LINK1, 12)
        self._draw_link(elbow_screen, ee_screen, COLOR_LINK2, 10)
        
        # Draw joints
        pygame.draw.circle(self.surface, COLOR_JOINT, base_screen, 10)
        pygame.draw.circle(self.surface, (30, 30, 30), base_screen, 10, 2)
        
        pygame.draw.circle(self.surface, COLOR_JOINT, elbow_screen, 8)
        pygame.draw.circle(self.surface, (30, 30, 30), elbow_screen, 8, 2)
        
        # Draw end-effector
        pygame.draw.circle(self.surface, COLOR_END_EFFECTOR, ee_screen, 8)
        pygame.draw.circle(self.surface, (255, 255, 255), ee_screen, 8, 2)
        
        # Draw tool orientation
        yaw = self.theta1 + self.theta2 + self.theta4
        tool_length = 0.03
        tool_end_x = ee_x + tool_length * math.cos(yaw)
        tool_end_y = ee_y + tool_length * math.sin(yaw)
        tool_end_screen = self.world_to_screen(tool_end_x, tool_end_y)
        pygame.draw.line(self.surface, COLOR_END_EFFECTOR, ee_screen, tool_end_screen, 3)
    
    def _draw_link(self, p1: Tuple[int, int], p2: Tuple[int, int], 
                   color: Tuple[int, int, int], width: int):
        """Draw a robot link with nice styling"""
        pygame.draw.line(self.surface, color, p1, p2, width)
        # Add outline
        pygame.draw.line(self.surface, (30, 30, 30), p1, p2, width + 2)
        pygame.draw.line(self.surface, color, p1, p2, width)
    
    def _draw_target(self):
        """Draw target position marker"""
        target_screen = self.world_to_screen(self.target_x, self.target_y)
        
        # Crosshair
        size = 15
        pygame.draw.line(self.surface, COLOR_TARGET, 
                        (target_screen[0] - size, target_screen[1]),
                        (target_screen[0] + size, target_screen[1]), 2)
        pygame.draw.line(self.surface, COLOR_TARGET,
                        (target_screen[0], target_screen[1] - size),
                        (target_screen[0], target_screen[1] + size), 2)
        
        # Circle
        pygame.draw.circle(self.surface, COLOR_TARGET, target_screen, 10, 2)


class Dashboard:
    """Information dashboard panel"""
    
    def __init__(self, surface: pygame.Surface, rect: pygame.Rect):
        self.surface = surface
        self.rect = rect
        
        pygame.font.init()
        self.font_title = pygame.font.SysFont('Consolas', 18, bold=True)
        self.font_label = pygame.font.SysFont('Consolas', 14)
        self.font_value = pygame.font.SysFont('Consolas', 16, bold=True)
        
        # Data
        self.joints: Dict[str, float] = {"theta1": 0, "theta2": 0, "d3": 0, "theta4": 0}
        self.pose: Dict[str, float] = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.mode: str = "idle"
        self.ik_valid: bool = True
        self.at_target: bool = True
        self.progress: float = 0.0
    
    def update(self, joints: Dict, pose: Dict, mode: str, 
               ik_valid: bool, at_target: bool, progress: float):
        """Update dashboard data"""
        self.joints = joints
        self.pose = pose
        self.mode = mode
        self.ik_valid = ik_valid
        self.at_target = at_target
        self.progress = progress
    
    def draw(self):
        """Draw the dashboard"""
        # Background
        pygame.draw.rect(self.surface, (30, 35, 40), self.rect)
        pygame.draw.rect(self.surface, (60, 65, 70), self.rect, 2)
        
        x = self.rect.x + 15
        y = self.rect.y + 15
        line_height = 22
        
        # Title
        title = self.font_title.render("SCARA Dashboard", True, (220, 220, 220))
        self.surface.blit(title, (x, y))
        y += line_height + 10
        
        # Mode status
        mode_color = {
            "idle": (150, 150, 150),
            "moving": (100, 200, 255),
            "reached": (100, 255, 100),
            "error": (255, 100, 100)
        }.get(self.mode, (200, 200, 200))
        
        mode_text = self.font_label.render(f"Mode: ", True, COLOR_TEXT)
        self.surface.blit(mode_text, (x, y))
        mode_val = self.font_value.render(self.mode.upper(), True, mode_color)
        self.surface.blit(mode_val, (x + 60, y))
        y += line_height
        
        # IK status
        ik_color = (100, 255, 100) if self.ik_valid else (255, 100, 100)
        ik_text = self.font_label.render("IK: ", True, COLOR_TEXT)
        self.surface.blit(ik_text, (x, y))
        ik_val = self.font_value.render("VALID" if self.ik_valid else "INVALID", True, ik_color)
        self.surface.blit(ik_val, (x + 60, y))
        y += line_height + 10
        
        # Progress bar
        prog_text = self.font_label.render("Progress:", True, COLOR_TEXT)
        self.surface.blit(prog_text, (x, y))
        y += line_height - 5
        bar_width = self.rect.width - 40
        bar_height = 10
        pygame.draw.rect(self.surface, (50, 55, 60), (x, y, bar_width, bar_height))
        pygame.draw.rect(self.surface, (100, 200, 255), 
                        (x, y, int(bar_width * self.progress), bar_height))
        pygame.draw.rect(self.surface, (80, 85, 90), (x, y, bar_width, bar_height), 1)
        y += bar_height + 15
        
        # Pose section
        pose_title = self.font_title.render("End-Effector Pose", True, (180, 180, 180))
        self.surface.blit(pose_title, (x, y))
        y += line_height + 5
        
        for key, unit in [("x", "m"), ("y", "m"), ("z", "m"), ("yaw", "rad")]:
            val = self.pose.get(key, 0)
            text = self.font_label.render(f"{key}: ", True, COLOR_TEXT)
            self.surface.blit(text, (x, y))
            val_text = self.font_value.render(f"{val:+.4f} {unit}", True, (255, 255, 255))
            self.surface.blit(val_text, (x + 45, y))
            y += line_height
        
        y += 10
        
        # Joints section
        joint_title = self.font_title.render("Joint State", True, (180, 180, 180))
        self.surface.blit(joint_title, (x, y))
        y += line_height + 5
        
        for key, unit in [("theta1", "rad"), ("theta2", "rad"), ("d3", "m"), ("theta4", "rad")]:
            val = self.joints.get(key, 0)
            
            # Convert radians to degrees for display
            if key != "d3":
                deg = math.degrees(val)
                text = self.font_label.render(f"{key}: ", True, COLOR_TEXT)
                self.surface.blit(text, (x, y))
                val_text = self.font_value.render(f"{deg:+7.2f}°", True, (255, 255, 255))
                self.surface.blit(val_text, (x + 65, y))
            else:
                text = self.font_label.render(f"{key}: ", True, COLOR_TEXT)
                self.surface.blit(text, (x, y))
                val_text = self.font_value.render(f"{val*1000:+7.1f} mm", True, (255, 255, 255))
                self.surface.blit(val_text, (x + 65, y))
            y += line_height

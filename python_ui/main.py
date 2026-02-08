"""
SCARA Robot Dashboard - Main Application
Run this to start the visualization dashboard
"""

import pygame
import sys
import time
import math
from typing import Optional

from comm_client import CommClient, Telemetry
from visualization import SCARAVisualization, Dashboard, COLOR_BACKGROUND, COLOR_TEXT

# Window configuration
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 700
FPS = 60

# Layout
VIS_AREA = pygame.Rect(0, 0, 750, 700)
DASHBOARD_AREA = pygame.Rect(750, 0, 250, 450)
INPUT_AREA = pygame.Rect(750, 450, 250, 250)

# SCARA configuration (match C code)
SCARA_L1 = 0.30
SCARA_L2 = 0.25
SCARA_D1 = 0.40


class InputPanel:
    """Input panel for sending commands"""
    
    def __init__(self, surface: pygame.Surface, rect: pygame.Rect):
        self.surface = surface
        self.rect = rect
        
        pygame.font.init()
        self.font_title = pygame.font.SysFont('Consolas', 16, bold=True)
        self.font_label = pygame.font.SysFont('Consolas', 14)
        self.font_input = pygame.font.SysFont('Consolas', 14)
        
        # Input fields
        self.fields = {
            "x": "0.40",
            "y": "0.20",
            "z": "0.35",
            "yaw": "0.50"
        }
        self.active_field: Optional[str] = None
        
        # Button rects (set in draw)
        self.go_button = pygame.Rect(0, 0, 0, 0)
        self.stop_button = pygame.Rect(0, 0, 0, 0)
        self.reset_button = pygame.Rect(0, 0, 0, 0)
        self.field_rects = {}
    
    def draw(self):
        """Draw input panel"""
        # Background
        pygame.draw.rect(self.surface, (25, 30, 35), self.rect)
        pygame.draw.rect(self.surface, (60, 65, 70), self.rect, 2)
        
        x = self.rect.x + 15
        y = self.rect.y + 15
        line_height = 30
        
        # Title
        title = self.font_title.render("Target Pose", True, (220, 220, 220))
        self.surface.blit(title, (x, y))
        y += line_height
        
        # Input fields
        field_width = 100
        label_width = 50
        
        for key in ["x", "y", "z", "yaw"]:
            # Label
            unit = "m" if key != "yaw" else "rad"
            label = self.font_label.render(f"{key} ({unit}):", True, COLOR_TEXT)
            self.surface.blit(label, (x, y + 3))
            
            # Input box
            input_rect = pygame.Rect(x + label_width + 20, y, field_width, 24)
            self.field_rects[key] = input_rect
            
            bg_color = (50, 55, 60) if self.active_field != key else (60, 80, 100)
            pygame.draw.rect(self.surface, bg_color, input_rect)
            pygame.draw.rect(self.surface, (100, 105, 110), input_rect, 1)
            
            # Value
            value_text = self.font_input.render(self.fields[key], True, (255, 255, 255))
            self.surface.blit(value_text, (input_rect.x + 5, input_rect.y + 4))
            
            y += line_height
        
        y += 10
        
        # Buttons
        button_width = 70
        button_height = 30
        button_spacing = 10
        
        # Go button
        self.go_button = pygame.Rect(x, y, button_width, button_height)
        pygame.draw.rect(self.surface, (40, 120, 80), self.go_button)
        pygame.draw.rect(self.surface, (60, 180, 120), self.go_button, 2)
        go_text = self.font_title.render("GO", True, (255, 255, 255))
        self.surface.blit(go_text, (self.go_button.centerx - go_text.get_width()//2,
                                    self.go_button.centery - go_text.get_height()//2))
        
        # Stop button
        self.stop_button = pygame.Rect(x + button_width + button_spacing, y, 
                                       button_width, button_height)
        pygame.draw.rect(self.surface, (150, 80, 40), self.stop_button)
        pygame.draw.rect(self.surface, (200, 120, 80), self.stop_button, 2)
        stop_text = self.font_title.render("STOP", True, (255, 255, 255))
        self.surface.blit(stop_text, (self.stop_button.centerx - stop_text.get_width()//2,
                                      self.stop_button.centery - stop_text.get_height()//2))
        
        # Reset button
        self.reset_button = pygame.Rect(x + 2*(button_width + button_spacing), y,
                                        button_width, button_height)
        pygame.draw.rect(self.surface, (80, 80, 100), self.reset_button)
        pygame.draw.rect(self.surface, (120, 120, 140), self.reset_button, 2)
        reset_text = self.font_title.render("RESET", True, (255, 255, 255))
        self.surface.blit(reset_text, (self.reset_button.centerx - reset_text.get_width()//2,
                                       self.reset_button.centery - reset_text.get_height()//2))
    
    def handle_click(self, pos: tuple) -> Optional[str]:
        """Handle mouse click, return action name or None"""
        # Check buttons
        if self.go_button.collidepoint(pos):
            return "go"
        if self.stop_button.collidepoint(pos):
            return "stop"
        if self.reset_button.collidepoint(pos):
            return "reset"
        
        # Check input fields
        for key, rect in self.field_rects.items():
            if rect.collidepoint(pos):
                self.active_field = key
                return None
        
        self.active_field = None
        return None
    
    def handle_key(self, event: pygame.event.Event):
        """Handle keyboard input"""
        if self.active_field is None:
            return
        
        if event.key == pygame.K_BACKSPACE:
            self.fields[self.active_field] = self.fields[self.active_field][:-1]
        elif event.key == pygame.K_RETURN:
            self.active_field = None
        elif event.key == pygame.K_TAB:
            # Cycle through fields
            keys = list(self.fields.keys())
            idx = keys.index(self.active_field)
            self.active_field = keys[(idx + 1) % len(keys)]
        elif event.unicode.isprintable():
            # Only allow valid number characters
            if event.unicode in "0123456789.-":
                self.fields[self.active_field] += event.unicode
    
    def get_target_pose(self) -> tuple:
        """Get target pose values"""
        try:
            x = float(self.fields["x"])
            y = float(self.fields["y"])
            z = float(self.fields["z"])
            yaw = float(self.fields["yaw"])
            return (x, y, z, yaw)
        except ValueError:
            return None


def main():
    """Main application entry point"""
    pygame.init()
    pygame.display.set_caption("SCARA Robot Dashboard")
    
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    clock = pygame.time.Clock()
    
    # Create visualization components
    vis = SCARAVisualization(screen, 
                             center=(VIS_AREA.width // 2, VIS_AREA.height // 2),
                             scale=450.0, L1=SCARA_L1, L2=SCARA_L2)
    
    dashboard = Dashboard(screen, DASHBOARD_AREA)
    input_panel = InputPanel(screen, INPUT_AREA)
    
    # Create communication client
    client = CommClient()
    connected = client.connect()
    
    # Font for status
    font_status = pygame.font.SysFont('Consolas', 14)
    
    # Main loop
    running = True
    last_telemetry_time = time.time()
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    action = input_panel.handle_click(event.pos)
                    
                    if action == "go":
                        target = input_panel.get_target_pose()
                        if target and connected:
                            x, y, z, yaw = target
                            client.send_goto_pose(x, y, z, yaw)
                            vis.set_target(x, y)
                    
                    elif action == "stop":
                        if connected:
                            client.send_stop()
                    
                    elif action == "reset":
                        if connected:
                            client.send_reset()
                            vis.set_target(None, None)
            
            elif event.type == pygame.KEYDOWN:
                input_panel.handle_key(event)
        
        # Get latest telemetry
        if connected:
            tel = client.get_telemetry()
            
            # Update visualization
            vis.update_state(
                tel.joints.get("theta1", 0),
                tel.joints.get("theta2", 0),
                tel.joints.get("d3", 0),
                tel.joints.get("theta4", 0)
            )
            
            # Update dashboard
            dashboard.update(
                tel.joints, tel.pose, tel.mode,
                tel.ik_valid, tel.at_target, tel.progress
            )
            
            # Clear target when reached
            if tel.mode == "reached":
                vis.set_target(None, None)
        
        # Draw
        screen.fill(COLOR_BACKGROUND)
        
        vis.draw()
        dashboard.draw()
        input_panel.draw()
        
        # Connection status
        status_color = (100, 255, 100) if connected else (255, 100, 100)
        status_text = "Connected" if connected else "Disconnected"
        status_surf = font_status.render(f"Status: {status_text}", True, status_color)
        screen.blit(status_surf, (10, WINDOW_HEIGHT - 25))
        
        # Try to reconnect if disconnected
        if not connected:
            if time.time() - last_telemetry_time > 2.0:
                connected = client.connect()
                last_telemetry_time = time.time()
        else:
            last_telemetry_time = time.time()
        
        pygame.display.flip()
        clock.tick(FPS)
    
    # Cleanup
    client.disconnect()
    pygame.quit()


if __name__ == "__main__":
    main()

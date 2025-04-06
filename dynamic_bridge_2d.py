import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from enum import Enum
from collections import deque
import copy
from datetime import datetime
import os

class ModuleType(Enum):
    """Enum for different types of modules in the system"""
    SOURCE = 1      # Source modules that need to move to targets
    TARGET = 2      # Target positions where source modules should go
    STRUCTURE = 3   # Static structure modules (part of the original configuration)
    UNUSED = 4      # Modules not initially part of any path (can form bridges)
    BRIDGE = 5      # Modules that have been repurposed as bridge components
    EMPTY = 6       # Empty space (no module)

class Module:
    """Class representing a single module in the 2D grid"""
    def __init__(self, id, x, y, module_type):
        self.id = id
        self.x = x
        self.y = y
        self.type = module_type
        self.occupied = module_type != ModuleType.EMPTY
        self.moving_module = None  # If this is a structure module, it may have a moving module on it
    
    def __repr__(self):
        return f"Module({self.id}, ({self.x}, {self.y}), {self.type})"
    
    def get_position(self):
        return (self.x, self.y)
    
    def is_adjacent_to(self, other_module):
        """Check if this module is adjacent to another module (including diagonals)"""
        dx = abs(self.x - other_module.x)
        dy = abs(self.y - other_module.y)
        return (dx <= 1 and dy <= 1) and (dx + dy > 0)  # Adjacent but not the same

class DynamicBridgeSystem:
    """Main class for the dynamic bridge 2D system"""
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[None for _ in range(width)] for _ in range(height)]
        self.modules = {}  # Dictionary of all modules by ID
        self.source_modules = []  # List of source modules
        self.target_positions = []  # List of target positions
        self.flow_graph = nx.DiGraph()  # Graph for flow calculations
        self.bridge_candidates = []  # List of unused modules that can form bridges
        
        # Super source and sink for flow calculations
        self.super_source = 'Super_S'
        self.super_sink = 'Super_T'
    
    def add_module(self, id, x, y, module_type):
        """Add a module to the system"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            raise ValueError(f"Position ({x}, {y}) is outside the grid")
        
        module = Module(id, x, y, module_type)
        self.grid[y][x] = module
        self.modules[id] = module
        
        # Track special module types
        if module_type == ModuleType.SOURCE:
            self.source_modules.append(module)
        elif module_type == ModuleType.TARGET:
            self.target_positions.append(module)
        elif module_type == ModuleType.UNUSED:
            self.bridge_candidates.append(module)
    
    def is_connected(self, module, exclude_module=None):
        """Check if a module is connected to the structure (has at least one neighbor)"""
        neighbors = self.get_adjacent_modules(module)
        
        # Filter out the excluded module if specified
        if exclude_module:
            neighbors = [n for n in neighbors if n.id != exclude_module.id]
        
        # Check if there's at least one non-empty adjacent module
        return any(n.type != ModuleType.EMPTY for n in neighbors)
    
    def get_adjacent_modules(self, module, include_diagonals=True):
        """Get all adjacent modules (including diagonals if specified)"""
        adjacent_modules = []
        
        # Define offsets for adjacent cells (8-connectivity if include_diagonals=True)
        offsets = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-connectivity (up, right, down, left)
        
        if include_diagonals:
            offsets.extend([(1, 1), (1, -1), (-1, -1), (-1, 1)])  # Add diagonal offsets
        
        for dx, dy in offsets:
            new_x, new_y = module.x + dx, module.y + dy
            
            # Check if position is within grid bounds
            if 0 <= new_x < self.width and 0 <= new_y < self.height:
                adj_module = self.grid[new_y][new_x]
                if adj_module is not None:
                    adjacent_modules.append(adj_module)
        
        return adjacent_modules
    
    def initialize_from_example(self):
        """Initialize the system with the example configuration from the image"""
        # Create a grid based on the image in the problem description
        # Red circles (sources) on the left, green circles (targets) on the right,
        # black outlined circles (structure modules) in the middle
        
        # Sources (left side, red interior)
        for y in range(5):
            self.add_module(f"S_{y}", 0, y, ModuleType.SOURCE)
        
        # Structure modules (white circles with black outline)
        # Left block
        for y in range(5):
            for x in range(1, 4):
                self.add_module(f"M_L_{y}_{x}", x, y, ModuleType.STRUCTURE)
        
        # Bridge in the middle (row 2)
        for x in range(4, 9):
            self.add_module(f"M_B_{x}", x, 2, ModuleType.STRUCTURE)
        
        # Right block
        for y in range(5):
            for x in range(9, 12):
                self.add_module(f"M_R_{y}_{x}", x, y, ModuleType.STRUCTURE)
        
        # Targets (right side, green outline)
        for y in range(5):
            self.add_module(f"T_{y}", 12, y, ModuleType.TARGET)
        
        # Identify modules that could be used to form bridges (initially UNUSED)
        # Top and bottom rows in the middle section (currently empty)
        for y in [0, 1, 3, 4]:  # Rows 0, 1, 3, 4 in the middle
            for x in range(4, 9):
                self.add_module(f"U_{y}_{x}", x, y, ModuleType.UNUSED)
        
        # Mark remaining spaces as empty
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] is None:
                    self.add_module(f"E_{y}_{x}", x, y, ModuleType.EMPTY)
    
    def visualize_grid(self, title="Dynamic Bridge System", show=True, save_path=None):
        """Visualize the current state of the grid"""
        fig, ax = plt.subplots(figsize=(15, 8))
        
        # Colors for different module types (fill and outline)
        fill_colors = {
            ModuleType.SOURCE: 'red',
            ModuleType.TARGET: 'white',  # Targets are white with green outline
            ModuleType.STRUCTURE: 'white',
            ModuleType.UNUSED: 'white',
            ModuleType.BRIDGE: 'orange',
            ModuleType.EMPTY: 'white'
        }
        
        edge_colors = {
            ModuleType.SOURCE: 'black',
            ModuleType.TARGET: 'green',
            ModuleType.STRUCTURE: 'black',
            ModuleType.UNUSED: 'gray',
            ModuleType.BRIDGE: 'black',
            ModuleType.EMPTY: 'none'  # No edge for empty space
        }
        
        # Flag for whether to fill the circle
        fill_module = {
            ModuleType.SOURCE: True,
            ModuleType.TARGET: False,
            ModuleType.STRUCTURE: False,
            ModuleType.UNUSED: False,
            ModuleType.BRIDGE: True,
            ModuleType.EMPTY: False
        }
        
        # Plotting each module
        for y in range(self.height):
            for x in range(self.width):
                module = self.grid[y][x]
                if module:
                    if module.type == ModuleType.EMPTY:
                        continue  # Skip drawing empty spaces
                    
                    # Base circle for the module
                    circle = plt.Circle(
                        (x, self.height - 1 - y), 
                        0.4, 
                        fill=fill_module[module.type],
                        edgecolor=edge_colors[module.type], 
                        facecolor=fill_colors[module.type],
                        linewidth=2
                    )
                    ax.add_patch(circle)
                    
                    # Add text label for ID (optional, can be commented out for cleaner visualization)
                    # plt.text(x, self.height - 1 - y, module.id, ha='center', va='center', fontsize=8)
        
        # Set plot properties
        ax.set_xlim(-1, self.width)
        ax.set_ylim(-1, self.height)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title(title)
        
        # Add a legend
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                      markeredgecolor='black', markersize=15, label='Source'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', 
                      markeredgecolor='green', markersize=15, label='Target'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', 
                      markeredgecolor='black', markersize=15, label='Structure'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', 
                      markeredgecolor='gray', markersize=15, label='Unused'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', 
                      markeredgecolor='black', markersize=15, label='Bridge')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
        # Save if path provided
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        # Show if requested
        if show:
            plt.show()
        else:
            plt.close()

# Example usage
if __name__ == "__main__":
    # Create a 13x5 grid for our example
    system = DynamicBridgeSystem(13, 5)
    
    # Initialize with the example configuration
    system.initialize_from_example()
    
    # Create images directory if it doesn't exist
    if not os.path.exists("images_2d"):
        os.makedirs("images_2d")
    
    # Visualize the initial state and save it
    system.visualize_grid("Initial Configuration", save_path="images_2d/initial_configuration.png")
    
    print("Grid initialized. Check the saved visualization in images_2d/initial_configuration.png")
    print("Press Enter to continue...")
    input()  # Wait for user input before closing 
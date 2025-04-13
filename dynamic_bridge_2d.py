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
        
        # Additional parameters
        self.allow_diagonal_moves = True  # Allow diagonal movement of modules
    
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
        """Initialize the system with the new configuration from the image"""
        # Create a grid based on the new image
        # We need a larger grid for this layout (8x13)
        self.width = 13
        self.height = 9
        self.grid = [[None for _ in range(self.width)] for _ in range(self.height)]
        
        # Sources (left side, red interior) - in the middle rows
        for y in range(2, 7):
            self.add_module(f"S_{y-2}", 0, y, ModuleType.SOURCE)
        
        # Structure modules (white circles with black outline)
        # Left column
        for y in range(9):
            for x in range(1, 5):
                self.add_module(f"M_L_{y}_{x}", x, y, ModuleType.STRUCTURE)
        
        # Right column
        for y in range(9):
            for x in range(8, 12):
                self.add_module(f"M_R_{y}_{x}", x, y, ModuleType.STRUCTURE)
        
        # Bridge in the middle (row 4, which is the 3rd row from the top of the sources)
        for x in range(5, 8):
            self.add_module(f"M_B_{x}", x, 4, ModuleType.STRUCTURE)
        
        # Targets (right side, green outline) - in the middle rows
        for y in range(2, 7):
            self.add_module(f"T_{y-2}", 12, y, ModuleType.TARGET)
        
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
            ModuleType.BRIDGE: 'orange',
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
                    
                    # Add a label for bridge modules
                    if module.type == ModuleType.BRIDGE:
                        plt.text(x, self.height - 1 - y, "B", ha='center', va='center', 
                               fontsize=10, fontweight='bold', color='black')
        
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
                      markeredgecolor='orange', markersize=15, label='Bridge')
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

    def build_flow_graph(self):
        """Build a flow graph for Edmonds-Karp algorithm"""
        # Create a fresh DiGraph
        self.flow_graph = nx.DiGraph()
        
        # Add nodes for all modules
        for module_id, module in self.modules.items():
            if module.type != ModuleType.EMPTY:
                self.flow_graph.add_node(module_id, pos=(module.x, module.y), type=module.type)
        
        # Add super source and sink
        self.flow_graph.add_node(self.super_source, pos=(-1, self.height//2), type='super_source')
        self.flow_graph.add_node(self.super_sink, pos=(self.width, self.height//2), type='super_sink')
        
        # Connect source modules to super source with capacity 1
        for source in self.source_modules:
            self.flow_graph.add_edge(self.super_source, source.id, capacity=1)
        
        # Connect target positions to super sink with capacity 1
        for target in self.target_positions:
            self.flow_graph.add_edge(target.id, self.super_sink, capacity=1)
        
        # Identify original bridge modules (in row 4)
        original_bridge_ids = ["M_B_5", "M_B_6", "M_B_7"]
        
        # Connect adjacent structure modules
        structure_modules = [m for m in self.modules.values() 
                            if m.type in [ModuleType.STRUCTURE, ModuleType.BRIDGE]]
        
        for i, module1 in enumerate(structure_modules):
            for module2 in structure_modules[i+1:]:
                if module1.is_adjacent_to(module2):
                    # Add bidirectional edges with capacity 10 (increased from 1)
                    self.flow_graph.add_edge(module1.id, module2.id, capacity=1)
                    self.flow_graph.add_edge(module2.id, module1.id, capacity=1)
        
        # Connect sources to adjacent structure modules
        for source in self.source_modules:
            adjacent_modules = self.get_adjacent_modules(source)
            for adj in adjacent_modules:
                if adj.type in [ModuleType.STRUCTURE, ModuleType.BRIDGE]:
                    self.flow_graph.add_edge(source.id, adj.id, capacity=1)  # Increased from 1
        
        # Connect targets to adjacent structure modules
        for target in self.target_positions:
            adjacent_modules = self.get_adjacent_modules(target)
            for adj in adjacent_modules:
                if adj.type in [ModuleType.STRUCTURE, ModuleType.BRIDGE]:
                    self.flow_graph.add_edge(adj.id, target.id, capacity=1)  # Increased from 1
        
        return self.flow_graph

    def calculate_max_flow(self):
        """Calculate maximum flow using Edmonds-Karp algorithm"""
        # Ensure flow graph is built
        if not self.flow_graph or len(self.flow_graph.edges) == 0:
            self.build_flow_graph()
        
        # Calculate maximum flow
        flow_value, flow_dict = nx.maximum_flow(
            self.flow_graph, self.super_source, self.super_sink, 
            flow_func=nx.algorithms.flow.edmonds_karp
        )
        
        # Update graph with flow values
        for u in flow_dict:
            for v, flow in flow_dict[u].items():
                self.flow_graph[u][v]['flow'] = flow
        
        return flow_value, flow_dict

    def identify_unused_modules(self):
        """Identify structure modules that are not used in any flow path"""
        # Ensure we have calculated the flow
        if not hasattr(self.flow_graph, 'edges') or not any('flow' in data for _, _, data in self.flow_graph.edges(data=True)):
            self.calculate_max_flow()
        
        # Reset the bridge candidates list
        self.bridge_candidates = []
        
        # Find structure modules with no flow
        for module_id, module in self.modules.items():
            if module.type == ModuleType.STRUCTURE:
                # Check if this module has any flow going through it
                has_flow = False
                
                # Check all edges connected to this module
                if module_id in self.flow_graph:
                    for _, v, data in self.flow_graph.out_edges(module_id, data=True):
                        if data.get('flow', 0) > 0:
                            has_flow = True
                            break
                
                # If no flow, mark as unused and add to bridge candidates
                if not has_flow:
                    module.type = ModuleType.UNUSED
                    self.bridge_candidates.append(module)
        
        return self.bridge_candidates

    def visualize_flow(self, title="Flow Visualization", show=True, save_path=None):
        """Visualize the flow graph with flow values"""
        if not hasattr(self.flow_graph, 'edges') or len(self.flow_graph.edges) == 0:
            print("Flow graph not built. Run calculate_max_flow() first.")
            return
        
        # Create figure
        fig, ax = plt.subplots(figsize=(15, 8))
        
        # Get positions from the grid
        pos = {}
        for node in self.flow_graph.nodes():
            if node == self.super_source:
                pos[node] = (-1, self.height//2)
            elif node == self.super_sink:
                pos[node] = (self.width, self.height//2)
            else:
                module = self.modules.get(node)
                if module:
                    pos[node] = (module.x, self.height - 1 - module.y)
        
        # Draw nodes with different colors based on type
        node_colors = []
        for node in self.flow_graph.nodes():
            if node == self.super_source:
                node_colors.append('gold')
            elif node == self.super_sink:
                node_colors.append('purple')
            elif node in self.modules:
                module = self.modules[node]
                if module.type == ModuleType.SOURCE:
                    node_colors.append('red')
                elif module.type == ModuleType.TARGET:
                    node_colors.append('green')
                elif module.type == ModuleType.STRUCTURE:
                    node_colors.append('black')
                elif module.type == ModuleType.UNUSED:
                    node_colors.append('gray')
                elif module.type == ModuleType.BRIDGE:
                    node_colors.append('orange')
                else:
                    node_colors.append('white')
        
        # Draw nodes
        nx.draw_networkx_nodes(self.flow_graph, pos, node_color=node_colors, 
                              node_size=500, alpha=0.8)
        
        # Draw edges with flow values
        edges_with_flow = [(u, v) for u, v, data in self.flow_graph.edges(data=True) 
                          if data.get('flow', 0) > 0]
        
        # Draw edges with flow (thicker, red)
        if edges_with_flow:
            edge_width = [self.flow_graph[u][v].get('flow', 0) * 3 for u, v in edges_with_flow]
            nx.draw_networkx_edges(self.flow_graph, pos, edgelist=edges_with_flow, 
                                  width=edge_width, edge_color='red', alpha=0.7)
        
        # Draw edges without flow (thin, gray)
        edges_no_flow = [(u, v) for u, v, data in self.flow_graph.edges(data=True) 
                        if data.get('flow', 0) == 0]
        if edges_no_flow:
            nx.draw_networkx_edges(self.flow_graph, pos, edgelist=edges_no_flow, 
                                  width=0.5, edge_color='gray', alpha=0.3, style='dashed')
        
        # Add edge labels (flow/capacity)
        edge_labels = {(u, v): f"{data.get('flow', 0)}/{data.get('capacity', 0)}" 
                      for u, v, data in self.flow_graph.edges(data=True)}
        nx.draw_networkx_edge_labels(self.flow_graph, pos, edge_labels=edge_labels, font_size=8)
        
        # Add node labels
        nx.draw_networkx_labels(self.flow_graph, pos, font_size=10)
        
        # Set plot properties
        ax.set_title(title)
        ax.axis('off')
        plt.tight_layout()
        
        # Save if path provided
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        # Show if requested
        if show:
            plt.show()
        else:
            plt.close()

    def is_valid_move(self, module, new_x, new_y):
        """
        Check if a module can move to a new position while maintaining connectivity.
        
        Args:
            module: The module to move
            new_x, new_y: The new position coordinates
            
        Returns:
            bool: True if the move is valid, False otherwise
        """
        # Check if the destination is within grid bounds
        if new_x < 0 or new_x >= self.width or new_y < 0 or new_y >= self.height:
            return False
        
        # Check if the destination is empty or has a suitable structure module
        dest_module = self.grid[new_y][new_x]
        if not dest_module or dest_module.type != ModuleType.EMPTY:
            if dest_module.type == ModuleType.STRUCTURE and not dest_module.moving_module:
                # Can move onto a structure module if it's not occupied
                pass
            else:
                return False
        
        # Check if the module would still be connected after the move
        # For this, we'll check if it would be adjacent to any module after the move
        original_pos = (module.x, module.y)
        
        # Temporarily move the module to check connectivity
        module.x, module.y = new_x, new_y
        
        # Check if the module would be connected in the new position
        is_connected = self.is_connected(module)
        
        # Move the module back to its original position
        module.x, module.y = original_pos
        
        return is_connected

    def get_possible_moves(self, module):
        """
        Get all possible valid moves for a module.
        
        Args:
            module: The module to check moves for
            
        Returns:
            list: List of valid (x, y) positions the module can move to
        """
        valid_moves = []
        
        # Check all adjacent positions (including diagonals if allowed)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                # Skip the current position
                if dx == 0 and dy == 0:
                    continue
                    
                # Skip diagonal moves if not allowed
                if abs(dx) == 1 and abs(dy) == 1 and not self.allow_diagonal_moves:
                    continue
                    
                new_x, new_y = module.x + dx, module.y + dy
                
                if self.is_valid_move(module, new_x, new_y):
                    valid_moves.append((new_x, new_y))
        
        return valid_moves

    def move_module(self, module, new_x, new_y):
        """
        Move a module to a new position.
        
        Args:
            module: The module to move
            new_x, new_y: The new position coordinates
            
        Returns:
            bool: True if the move was successful, False otherwise
        """
        # Verify the move is valid
        if not self.is_valid_move(module, new_x, new_y):
            return False
        
        # Record the original position and type
        old_x, old_y = module.x, module.y
        old_type = module.type
        
        # Create an empty module for the old position if needed
        if f"E_{old_y}_{old_x}" not in self.modules:
            empty_module = Module(f"E_{old_y}_{old_x}", old_x, old_y, ModuleType.EMPTY)
            self.modules[f"E_{old_y}_{old_x}"] = empty_module
        
        # Check destination type
        dest_module = self.grid[new_y][new_x]
        
        if dest_module.type == ModuleType.EMPTY:
            # Moving to an empty space - update the grid references
            
            # Update the grid at the old position
            self.grid[old_y][old_x] = self.modules[f"E_{old_y}_{old_x}"]
            
            # Update the grid at the new position
            self.grid[new_y][new_x] = module
            
            # Update the module's coordinates and potentially type
            module.x, module.y = new_x, new_y
            
            # If this is an unused module being moved, change it to a bridge module
            if module.type == ModuleType.UNUSED:
                module.type = ModuleType.BRIDGE
            
        elif dest_module.type == ModuleType.STRUCTURE:
            # Moving onto a structure module - set the moving_module property
            dest_module.moving_module = module
            
            # Update the grid at the old position
            self.grid[old_y][old_x] = self.modules[f"E_{old_y}_{old_x}"]
            
            # Update the module's coordinates
            module.x, module.y = new_x, new_y
            
            # If this is an unused module being moved, change it to a bridge module
            if module.type == ModuleType.UNUSED:
                module.type = ModuleType.BRIDGE
        
        # Debug information
        print(f"Moved module from ({old_x}, {old_y}) to ({new_x}, {new_y})")
        print(f"Module type changed from {old_type} to {module.type}")
        
        # Return success
        return True

    def find_path(self, start_module, target_x, target_y, max_path_length=20):
        """
        Find a path from a module to a target position using BFS.
        
        Args:
            start_module: The starting module
            target_x, target_y: The target position coordinates
            max_path_length: Maximum path length to consider
            
        Returns:
            list: A list of (x, y) positions forming the path, or None if no path exists
        """
        # Initialize BFS data structures
        queue = deque([(start_module.x, start_module.y, [])])  # (x, y, path)
        visited = set([(start_module.x, start_module.y)])
        
        while queue:
            x, y, path = queue.popleft()
            
            # Check if we've reached the target
            if x == target_x and y == target_y:
                return path + [(x, y)]
            
            # Check if we've exceeded the maximum path length
            if len(path) >= max_path_length:
                continue
            
            # Try all possible moves from this position
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    # Skip the current position
                    if dx == 0 and dy == 0:
                        continue
                        
                    # Skip diagonal moves if not allowed
                    if abs(dx) == 1 and abs(dy) == 1 and not self.allow_diagonal_moves:
                        continue
                    
                    new_x, new_y = x + dx, y + dy
                    
                    # Skip if already visited or out of bounds
                    if (new_x, new_y) in visited or new_x < 0 or new_x >= self.width or new_y < 0 or new_y >= self.height:
                        continue
                    
                    # Check if the position is valid to move to
                    temp_module = copy.deepcopy(self.modules[f"M_L_0_0"])  # Create a temporary module for testing
                    temp_module.x, temp_module.y = x, y
                    
                    if self.is_valid_move(temp_module, new_x, new_y):
                        queue.append((new_x, new_y, path + [(x, y)]))
                        visited.add((new_x, new_y))
        
        # No path found
        return None

    def detect_bottlenecks(self):
        """
        Detect bottlenecks in the flow network.
        
        Returns:
            list: List of module IDs that are bottlenecks
        """
        # Ensure flow has been calculated
        if not hasattr(self.flow_graph, 'edges') or not any('flow' in data for _, _, data in self.flow_graph.edges(data=True)):
            self.calculate_max_flow()
        
        bottlenecks = []
        
        # Find edges that are at capacity
        for u, v, data in self.flow_graph.edges(data=True):
            if 'flow' in data and 'capacity' in data:
                if data['flow'] == data['capacity'] and data['flow'] > 0:
                    # This edge is at capacity - could be a bottleneck
                    if u != self.super_source and v != self.super_sink:
                        # Check if removing this edge would reduce the max flow
                        temp_graph = self.flow_graph.copy()
                        temp_graph[u][v]['capacity'] = 0
                        
                        # Recalculate flow with this edge disabled
                        temp_flow, _ = nx.maximum_flow(temp_graph, self.super_source, self.super_sink, 
                                                    flow_func=nx.algorithms.flow.edmonds_karp)
                        
                        # If the flow decreased, this is a bottleneck
                        current_flow_value = sum(data.get('flow', 0) for _, _, data in self.flow_graph.out_edges(self.super_source, data=True))
                        if temp_flow < current_flow_value:
                            bottlenecks.append((u, v))
        
        # Convert edge bottlenecks to module bottlenecks
        bottleneck_modules = set()
        for u, v in bottlenecks:
            if u in self.modules:
                bottleneck_modules.add(u)
            if v in self.modules:
                bottleneck_modules.add(v)
        
        return list(bottleneck_modules)

    def verify_bridge_placement(self, bridge_positions):
        """
        Verify if placing bridge modules at the given positions would create valid paths.
        
        Args:
            bridge_positions: List of (x, y) positions for the bridge
            
        Returns:
            bool: True if the bridge would connect left and right structure blocks
        """
        # Get the leftmost and rightmost positions of the bridge
        leftmost = min(bridge_positions, key=lambda pos: pos[0])
        rightmost = max(bridge_positions, key=lambda pos: pos[0])
        
        # Check if the leftmost bridge position is adjacent to the left structure block
        left_x, left_y = leftmost
        left_adjacent = False
        
        # Check all adjacent positions to the leftmost bridge position
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
            adj_x, adj_y = left_x + dx, left_y + dy
            
            # Check if this position is within bounds
            if 0 <= adj_x < self.width and 0 <= adj_y < self.height:
                adj_module = self.grid[adj_y][adj_x]
                # Check if it's a structure module in the left block (x < 5)
                if adj_module and adj_module.type == ModuleType.STRUCTURE and adj_module.x < 5:
                    left_adjacent = True
                    break
        
        # Check if the rightmost bridge position is adjacent to the right structure block
        right_x, right_y = rightmost
        right_adjacent = False
        
        # Check all adjacent positions to the rightmost bridge position
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
            adj_x, adj_y = right_x + dx, right_y + dy
            
            # Check if this position is within bounds
            if 0 <= adj_x < self.width and 0 <= adj_y < self.height:
                adj_module = self.grid[adj_y][adj_x]
                # Check if it's a structure module in the right block (x >= 8)
                if adj_module and adj_module.type == ModuleType.STRUCTURE and adj_module.x >= 8:
                    right_adjacent = True
                    break
        
        # The bridge is valid if it connects both left and right structure blocks
        return left_adjacent and right_adjacent
    
    def create_connected_bridge(self, row, min_x=5, max_x=7):
        """
        Create a bridge in the specified row that connects to both structure blocks.
        
        Args:
            row: The row to create the bridge in
            min_x, max_x: The x-coordinate range for the bridge
            
        Returns:
            list: List of bridge modules that were created, or empty list if unsuccessful
        """
        # Define the positions for the bridge
        bridge_positions = [(x, row) for x in range(min_x, max_x + 1)]
        
        # Verify if these positions would create a valid bridge
        if not self.verify_bridge_placement(bridge_positions):
            print(f"Cannot create a bridge in row {row} - it would not connect properly")
            return []
        
        # Check if all positions are empty
        for x, y in bridge_positions:
            if self.grid[y][x].type != ModuleType.EMPTY:
                print(f"Cannot create a bridge in row {row} - position ({x}, {y}) is not empty")
                return []
        
        # Find unused modules to move to these positions
        unused_modules = [m for m in self.bridge_candidates if m.type == ModuleType.UNUSED]
        
        # Check if we have enough unused modules
        if len(unused_modules) < len(bridge_positions):
            print(f"Not enough unused modules to create a bridge in row {row}")
            return []
        
        # Move unused modules to bridge positions
        bridge_modules = []
        for i, (x, y) in enumerate(bridge_positions):
            module = unused_modules[i]
            print(f"Moving module {module.id} from ({module.x}, {module.y}) to ({x}, {y})...")
            
            if self.move_module(module, x, y):
                bridge_modules.append(module)
                # Ensure it's marked as bridge type
                module.type = ModuleType.BRIDGE
                
                # Visualize after each movement to show the progress
                self.visualize_grid(f"After Moving Module to Position ({x}, {y})", 
                                   save_path=f"images_2d/simulation/05_movement_{i+1}.png",
                                   show=False)
            else:
                print(f"Failed to move module to ({x}, {y})")
        
        # If we successfully moved all modules, validate the bridge
        if len(bridge_modules) == len(bridge_positions):
            print(f"Successfully created a bridge in row {row}")
            
            # Rebuild the flow graph to include the new bridge
            self.flow_graph = nx.DiGraph()
            self.build_flow_graph()
            
            # Calculate the new flow to see if it increased
            flow_value, _ = self.calculate_max_flow()
            print(f"Flow value after bridge creation: {flow_value}")
            
            # Visualize the flow with the new bridge
            self.visualize_flow(f"Flow After Bridge in Row {row}", 
                               save_path=f"images_2d/simulation/flow_after_bridge_row_{row}.png",
                               show=False)
            
            return bridge_modules
        
        return []

    def verify_path_exists(self, source, target):
        """
        Verify if a path exists from a source to a target in the flow graph.
        
        Args:
            source: Source module
            target: Target module
            
        Returns:
            bool: True if a path exists, False otherwise
        """
        # Ensure flow graph is built
        if not self.flow_graph or len(self.flow_graph.edges) == 0:
            self.build_flow_graph()
        
        # Check if there's a path from source to target
        return nx.has_path(self.flow_graph, source.id, target.id)
    
    def verify_all_paths(self):
        """
        Verify if paths exist from all sources to all targets.
        
        Returns:
            dict: Dictionary mapping (source_id, target_id) to boolean path existence
        """
        # Ensure flow graph is built
        if not self.flow_graph or len(self.flow_graph.edges) == 0:
            self.build_flow_graph()
        
        path_exists = {}
        
        # Check all source-target pairs
        for source in self.source_modules:
            for target in self.target_positions:
                path_exists[(source.id, target.id)] = self.verify_path_exists(source, target)
        
        # Print summary
        valid_paths = sum(1 for exists in path_exists.values() if exists)
        total_paths = len(path_exists)
        print(f"Found {valid_paths} valid paths out of {total_paths} possible source-target pairs")
        
        return path_exists

    def are_positions_empty(self, positions):
        """
        Check if all given positions are empty and within grid bounds.
        
        Args:
            positions: List of (x, y) coordinate tuples to check
            
        Returns:
            bool: True if all positions are empty, False otherwise
        """
        for x, y in positions:
            # Check if position is within grid bounds
            if not (0 <= x < self.width and 0 <= y < self.height):
                return False
                
            # Check if position is empty
            if self.grid[y][x].type != ModuleType.EMPTY:
                return False
                
        return True
    
    def evaluate_bridge_location(self, bridge_positions):
        """
        Simulate placing a bridge at the given positions and evaluate flow improvement.
        
        Args:
            bridge_positions: List of (x, y) positions for the proposed bridge
            
        Returns:
            float: Flow improvement (new_flow - current_flow), or -1 if invalid
        """
        # Validate input
        if not bridge_positions:
            return -1
            
        # Calculate current flow as baseline
        current_flow, _ = self.calculate_max_flow()
        
        # Check if positions are valid for bridge placement
        if not self.verify_bridge_placement(bridge_positions) or not self.are_positions_empty(bridge_positions):
            return -1
            
        # Create a temporary copy of the system for simulation
        temp_system = copy.deepcopy(self)
        
        # Find unused modules to simulate bridge creation
        unused_modules = [m for m in temp_system.bridge_candidates if m.type == ModuleType.UNUSED]
        
        # Check if we have enough unused modules
        if len(unused_modules) < len(bridge_positions):
            return -1
            
        # Place bridge modules in the temporary system
        for i, (x, y) in enumerate(bridge_positions):
            module = unused_modules[i]
            # Simply teleport the module (don't need to simulate movement for evaluation)
            module.x, module.y = x, y
            module.type = ModuleType.BRIDGE
            temp_system.grid[y][x] = module
            
        # Rebuild flow graph in temporary system
        temp_system.flow_graph = nx.DiGraph()
        temp_system.build_flow_graph()
        
        # Calculate new flow
        new_flow, _ = temp_system.calculate_max_flow()
        
        # Return improvement
        return new_flow - current_flow

    def iterative_bottleneck_optimization(self, max_iterations=5, min_improvement=0, visualization_dir=None):
        """
        Iteratively identify bottlenecks and create bridges to address them.
        
        This method implements an adaptive optimization approach that:
        1. Detects current bottlenecks in the flow network
        2. Evaluates potential bridges around each bottleneck
        3. Creates the bridge with the highest expected flow improvement
        4. Repeats until no more beneficial bridges can be created
        
        Args:
            max_iterations: Maximum number of optimization iterations
            min_improvement: Minimum required flow improvement to continue
            visualization_dir: Directory to save visualization images (optional)
            
        Returns:
            tuple: (created_bridges, total_improvement)
                - created_bridges: List of dictionaries with bridge information
                - total_improvement: Overall flow improvement achieved
        """
        initial_flow, _ = self.calculate_max_flow()
        created_bridges = []
        current_flow = initial_flow
        iteration_visualizations = []
        
        print(f"Starting iterative optimization with initial flow: {initial_flow}")
        
        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration+1}/{max_iterations} ---")
            
            # 1. Detect bottlenecks
            bottlenecks = self.detect_bottlenecks()
            if not bottlenecks:
                print("No bottlenecks detected - network is optimized!")
                break
                
            print(f"Detected bottlenecks: {bottlenecks}")
            
            # 2. For each bottleneck, identify potential bypass options
            best_bridge = None
            best_improvement = 0
            
            for bottleneck_id in bottlenecks:
                bottleneck = self.modules.get(bottleneck_id)
                if not bottleneck:
                    continue
                    
                print(f"Analyzing bottleneck: {bottleneck_id} at ({bottleneck.x}, {bottleneck.y})")
                
                # Find rows adjacent to bottleneck for potential bridges
                for offset in [-1, 1]:  # Check one row above and below
                    row = bottleneck.y + offset
                    
                    if 0 <= row < self.height:  # Check row is within grid bounds
                        # Create potential bridge positions
                        bridge_positions = [(x, row) for x in range(5, 8)]
                        
                        # 3. Evaluate potential bridge at this location
                        improvement = self.evaluate_bridge_location(bridge_positions)
                        
                        print(f"  - Row {row}: Expected improvement = {improvement}")
                        
                        if improvement > best_improvement:
                            best_improvement = improvement
                            best_bridge = {'row': row, 'positions': bridge_positions, 'improvement': improvement}
            
            # 4. If we found a beneficial bridge, create it
            if best_bridge and best_improvement > min_improvement:
                print(f"\nCreating bridge in row {best_bridge['row']} for expected improvement of {best_bridge['improvement']}")
                
                # Actually create the bridge
                bridge_modules = self.create_connected_bridge(best_bridge['row'])
                
                if bridge_modules:
                    bridge_info = {
                        'row': best_bridge['row'],
                        'modules': [m.id for m in bridge_modules],
                        'improvement': best_improvement,
                        'iteration': iteration + 1
                    }
                    created_bridges.append(bridge_info)
                    
                    # Update current flow
                    current_flow, _ = self.calculate_max_flow()
                    actual_improvement = current_flow - (initial_flow + sum(b['improvement'] for b in created_bridges[:-1]))
                    print(f"New flow after bridge creation: {current_flow}")
                    print(f"Actual improvement: {actual_improvement} (expected: {best_improvement})")
                    
                    # Save visualization if directory provided
                    if visualization_dir:
                        os.makedirs(visualization_dir, exist_ok=True)
                        vis_path = f"{visualization_dir}/iteration_{iteration+1}_bridge_row_{best_bridge['row']}.png"
                        self.visualize_grid(f"After Bridge in Row {best_bridge['row']} (Iteration {iteration+1})", 
                                          save_path=vis_path, show=False)
                        
                        # Visualize flow with new bridge
                        flow_vis_path = f"{visualization_dir}/iteration_{iteration+1}_flow.png"
                        self.visualize_flow(f"Flow After Iteration {iteration+1}", 
                                         save_path=flow_vis_path, show=False)
                        
                        iteration_visualizations.append((vis_path, flow_vis_path))
                else:
                    print("Failed to create bridge - breaking optimization loop")
                    break
            else:
                print(f"No beneficial bridges found (best improvement: {best_improvement})")
                break
        
        total_improvement = current_flow - initial_flow
        print(f"\nOptimization complete:")
        print(f"- Initial flow: {initial_flow}")
        print(f"- Final flow: {current_flow}")
        print(f"- Total improvement: +{total_improvement}")
        print(f"- Bridges created: {len(created_bridges)}")
        
        return created_bridges, total_improvement

# Example usage
if __name__ == "__main__":
    # Create a 13x9 grid for our example
    system = DynamicBridgeSystem(13, 9)
    
    # Initialize with the example configuration
    system.initialize_from_example()
    
    # Create images directory if it doesn't exist
    if not os.path.exists("images_2d"):
        os.makedirs("images_2d")
    
    # Create simulation directory for step-by-step images
    simulation_dir = "images_2d/simulation"
    if not os.path.exists(simulation_dir):
        os.makedirs(simulation_dir)
    
    # Visualize the initial state and save it
    system.visualize_grid("Initial Configuration", 
                       save_path=f"{simulation_dir}/01_initial_configuration.png")
    
    print("Grid initialized. Check the saved visualization in images_2d/simulation/")
    
    # Calculate maximum flow
    print("Calculating maximum flow...")
    flow_value, flow_dict = system.calculate_max_flow()
    print(f"Maximum flow value: {flow_value}")
    
    # Visualize the flow
    system.visualize_flow("Initial Flow Network", 
                       save_path=f"{simulation_dir}/02_initial_flow.png")
    
    # Identify unused modules
    print("Identifying unused modules...")
    unused_modules = system.identify_unused_modules()
    print(f"Found {len(unused_modules)} unused modules that can be repurposed for bridges")
    
    # Visualize the grid with unused modules highlighted
    system.visualize_grid("Configuration with Unused Modules Identified", 
                       save_path=f"{simulation_dir}/03_unused_modules_identified.png")
    
    # Verify initial path connectivity
    print("\nVerifying initial path connectivity...")
    initial_paths = system.verify_all_paths()
    
    # Create a directory for optimization iterations
    optimization_dir = f"{simulation_dir}/optimization"
    if not os.path.exists(optimization_dir):
        os.makedirs(optimization_dir)
    
    # Run iterative bottleneck optimization
    print("\nStarting iterative bottleneck optimization...")
    created_bridges, improvement = system.iterative_bottleneck_optimization(
        max_iterations=3,
        min_improvement=0,
        visualization_dir=optimization_dir
    )
    
    # Final visualization showing all bridges
    system.visualize_grid("Final Configuration with Optimized Bridges", 
                       save_path=f"{simulation_dir}/06_final_bridges.png")
    
    # Visualize the improved flow
    system.visualize_flow("Final Flow Network with Optimized Bridges", 
                       save_path=f"{simulation_dir}/07_flow_with_bridges.png")
    
    # Generate a summary of created bridges
    print("\nBridge Creation Summary:")
    for i, bridge in enumerate(created_bridges):
        print(f"Bridge {i+1} in row {bridge['row']} (Iteration {bridge['iteration']}):")
        print(f"- Module IDs: {bridge['modules']}")
        print(f"- Expected improvement: {bridge['improvement']}")
    
    print("\nAnalysis and bridge construction complete.")
    print(f"Maximum flow increased from {flow_value} to {flow_value + improvement}")
    print("Check the saved visualizations in the images_2d/simulation directory.")
    print("Press Enter to continue...")
    input()  # Wait for user input before closing 
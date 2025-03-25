import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from networkx.algorithms.flow import edmonds_karp
import os
from datetime import datetime

class ModularRobot3D:
    """
    Class representing a 3D modular robot configuration as a flow network.
    """
    def __init__(self, name):
        self.name = name
        self.graph = nx.DiGraph()
        self.modules = {}  # Dict mapping module IDs to (position, type, capacity)
        self.source = 'Super_S'
        self.sink = 'Super_T'
        self.graph.add_node(self.source, pos=(0, 0, 0), type='super_source')
        self.graph.add_node(self.sink, pos=(10, 10, 10), type='super_sink')
        
        # Normalize name for file paths
        self.config_type = name.lower().replace(' ', '_')
    
    def add_module(self, module_id, position, module_type, capacity=1000):
        """
        Add a module to the configuration.
        
        Args:
            module_id: Unique identifier for the module
            position: (x, y, z) coordinates
            module_type: 'source', 'target', or 'intermediate'
            capacity: Flow capacity of module connections
        """
        self.modules[module_id] = (position, module_type, capacity)
        self.graph.add_node(module_id, pos=position, type=module_type)
        
        # Connect sources to super source and targets to super sink
        if module_type == 'source':
            self.graph.add_edge(self.source, module_id, capacity=float('inf'))
        elif module_type == 'target':
            self.graph.add_edge(module_id, self.sink, capacity=float('inf'))
    
    def connect_adjacent_modules(self, distance_threshold=1.5):
        """
        Connect modules that are adjacent to each other based on a distance threshold.
        In a grid-based system, direct neighbors would have a distance of 1.0.
        """
        module_ids = list(self.modules.keys())
        
        for i, id1 in enumerate(module_ids):
            pos1, type1, cap1 = self.modules[id1]
            
            for id2 in module_ids[i+1:]:
                pos2, type2, cap2 = self.modules[id2]
                
                # Calculate Euclidean distance
                distance = np.sqrt(sum((p1 - p2)**2 for p1, p2 in zip(pos1, pos2)))
                
                # Connect if they are adjacent
                if distance <= distance_threshold:
                    # Bidirectional connection with the specified capacity
                    self.graph.add_edge(id1, id2, capacity=cap1)
                    self.graph.add_edge(id2, id1, capacity=cap2)
    
    def add_explicit_connection(self, from_id, to_id, capacity=None):
        """Add an explicit directed connection between modules."""
        if from_id not in self.modules or to_id not in self.modules:
            raise ValueError("Module IDs must exist in the configuration")
        
        if capacity is None:
            # Use the capacity of the source module
            capacity = self.modules[from_id][2]
        
        self.graph.add_edge(from_id, to_id, capacity=capacity)
    
    def calculate_max_flow(self):
        """Calculate the maximum flow through the network."""
        flow_value, flow_dict = nx.maximum_flow(
            self.graph, self.source, self.sink, flow_func=edmonds_karp
        )
        
        # Update graph with flow values
        for u in flow_dict:
            for v, flow in flow_dict[u].items():
                self.graph[u][v]['flow'] = flow
        
        return flow_value, flow_dict
    
    def visualize_3d(self, filename=None, subfolder=None):
        """
        Create a 3D visualization of the modular robot configuration.
        
        Args:
            filename: If provided, save the visualization to this file
            subfolder: Specific subfolder in images_3d/ to save the file
        """
        fig = plt.figure(figsize=(14, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # Node positions
        pos = nx.get_node_attributes(self.graph, 'pos')
        
        # Create a mapping of node types to colors
        type_colors = {
            'super_source': 'gold',
            'super_sink': 'purple',
            'source': 'green',
            'target': 'cyan',
            'intermediate': 'gray'
        }
        
        # Group nodes by type
        node_types = nx.get_node_attributes(self.graph, 'type')
        nodes_by_type = {t: [] for t in type_colors}
        for node, ntype in node_types.items():
            nodes_by_type[ntype].append(node)
        
        # Calculate bounds for better view
        all_positions = [pos for node, pos in pos.items() if node != self.source and node != self.sink]
        if all_positions:
            x_vals = [p[0] for p in all_positions]
            y_vals = [p[1] for p in all_positions]
            z_vals = [p[2] for p in all_positions]
            
            x_min, x_max = min(x_vals), max(x_vals)
            y_min, y_max = min(y_vals), max(y_vals)
            z_min, z_max = min(z_vals), max(z_vals)
            
            # Set axis limits with padding
            padding = 1
            ax.set_xlim(x_min - padding, x_max + padding)
            ax.set_ylim(y_min - padding, y_max + padding)
            ax.set_zlim(z_min - padding, z_max + padding)
        
        # Draw nodes by type
        for ntype, color in type_colors.items():
            nodes = nodes_by_type[ntype]
            if not nodes:
                continue
                
            xs = [pos[n][0] for n in nodes if n in pos and n != self.source and n != self.sink]
            ys = [pos[n][1] for n in nodes if n in pos and n != self.source and n != self.sink]
            zs = [pos[n][2] for n in nodes if n in pos and n != self.source and n != self.sink]
            
            # Skip if no nodes to draw
            if not xs:
                continue
            
            # Draw nodes as spheres, larger size
            ax.scatter(xs, ys, zs, color=color, s=300, label=ntype, alpha=0.8)
            
            # Add node labels with clearer font
            for n, x, y, z in zip(nodes, xs, ys, zs):
                if n != self.source and n != self.sink:  # Skip labels for super nodes
                    ax.text(x, y, z, n, fontsize=9, fontweight='bold', 
                           ha='center', va='center')
        
        # Check if any edges have flow data
        has_flow = False
        for _, _, data in self.graph.edges(data=True):
            if 'flow' in data:
                has_flow = True
                break
        
        # Draw edges with flow values
        for u, v, data in self.graph.edges(data=True):
            if u in pos and v in pos:  # Only draw edges between positioned nodes
                # Skip edges connected to super nodes for clarity
                if u == self.source or v == self.sink:
                    continue
                    
                x = [pos[u][0], pos[v][0]]
                y = [pos[u][1], pos[v][1]]
                z = [pos[u][2], pos[v][2]]
                
                # Color edges based on flow (if calculated)
                if has_flow and 'flow' in data:
                    # Normalize flow for color intensity
                    flow_ratio = data['flow'] / data['capacity'] if data['capacity'] > 0 else 0
                    # Red for high flow, blue for low flow
                    edge_color = (1, 0, 0, 0.8) if flow_ratio > 0 else (0, 0, 1, 0.2)
                    
                    # Skip edges with no flow for clarity
                    if flow_ratio == 0:
                        continue
                    
                    # Thicker lines for edges with flow
                    linewidth = 3 * max(0.5, flow_ratio)
                else:
                    edge_color = (0, 0, 0, 0.3)  # Light gray for no flow data
                    linewidth = 1
                
                # Draw the edge
                ax.plot(x, y, z, color=edge_color, linewidth=linewidth)
                
                # Add flow/capacity label at midpoint with better visibility
                if has_flow and 'flow' in data and data['flow'] > 0:
                    mid_x, mid_y, mid_z = [(a + b)/2 for a, b in zip(pos[u], pos[v])]
                    ax.text(mid_x, mid_y, mid_z, f"{data['flow']}/{data['capacity']}", 
                           fontsize=10, fontweight='bold', color='darkred',
                           ha='center', va='center', bbox=dict(facecolor='white', alpha=0.5))
        
        # Set labels and title
        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        ax.set_zlabel('Z', fontsize=12)
        ax.set_title(f"{self.name} - 3D Modular Robot Configuration", fontsize=14)
        
        # Add legend with clearer placement
        ax.legend(loc='upper right', fontsize=10)
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
        
        # Adjust camera angle for better view
        ax.view_init(elev=30, azim=45)
        
        # Save if filename provided
        if filename:
            # Create timestamp for unique filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Determine save path
            if subfolder:
                # Make sure directory exists
                save_dir = os.path.join("images_3d", subfolder)
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                # Add timestamp to filename to avoid overwrites
                base_name = os.path.splitext(filename)[0]
                ext = os.path.splitext(filename)[1] or ".png"
                save_path = os.path.join(save_dir, f"{base_name}_{timestamp}{ext}")
            else:
                # Save in main images_3d folder if no subfolder specified
                if not os.path.exists("images_3d"):
                    os.makedirs("images_3d")
                
                # Add timestamp to filename to avoid overwrites
                base_name = os.path.splitext(filename)[0]
                ext = os.path.splitext(filename)[1] or ".png"
                save_path = os.path.join("images_3d", f"{base_name}_{timestamp}{ext}")
            
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.tight_layout()
        plt.show()

# Simple example: Creating a basic 3D grid configuration
def create_simple_grid_example():
    # Create a smaller 2x2x3 grid with sources at bottom and targets at top
    robot = ModularRobot3D("Simple 3D Grid")
    
    # Add source modules at the bottom layer (z=0)
    for x in range(2):
        for y in range(2):
            robot.add_module(f"S_{x}_{y}", (x, y, 0), 'source')
    
    # Add intermediate modules in the middle layer (z=1)
    for x in range(2):
        for y in range(2):
            robot.add_module(f"I_{x}_{y}", (x, y, 1), 'intermediate')
    
    # Add target modules at the top layer (z=2)
    for x in range(2):
        for y in range(2):
            robot.add_module(f"T_{x}_{y}", (x, y, 2), 'target')
    
    # Connect adjacent modules
    robot.connect_adjacent_modules()
    
    return robot

# Example with a small bridge configuration
def create_bridge_example():
    # Create a bridge-like structure with sources on one side and targets on the other
    robot = ModularRobot3D("Bridge Configuration")
    
    # Add source modules on one side
    for z in range(3):
        robot.add_module(f"S_0_{z}", (0, 0, z), 'source')
    
    # Add intermediate modules forming a bridge
    for x in range(1, 4):
        robot.add_module(f"I_{x}_1", (x, 0, 1), 'intermediate')
    
    # Add target modules on the other side
    for z in range(3):
        robot.add_module(f"T_4_{z}", (4, 0, z), 'target')
    
    # Connect adjacent modules
    robot.connect_adjacent_modules()
    
    return robot

# Simple vertical stack example
def create_stack_example():
    # Create a simple vertical stack with source at bottom, target at top
    robot = ModularRobot3D("Stack Configuration")
    
    # Add a source module at the bottom
    robot.add_module("Source", (0, 0, 0), 'source')
    
    # Add three intermediate modules
    robot.add_module("Middle1", (0, 0, 1), 'intermediate')
    robot.add_module("Middle2", (0, 0, 2), 'intermediate')
    robot.add_module("Middle3", (0, 0, 3), 'intermediate')
    
    # Add a target module at the top
    robot.add_module("Target", (0, 0, 4), 'target')
    
    # Connect the modules in a vertical stack
    robot.add_explicit_connection("Source", "Middle1")
    robot.add_explicit_connection("Middle1", "Middle2")
    robot.add_explicit_connection("Middle2", "Middle3")
    robot.add_explicit_connection("Middle3", "Target")
    
    return robot

if __name__ == "__main__":
    # Run only the stack example and image-like examples to save time
    
    # Create and visualize the stack example first (simplest)
    stack_robot = create_stack_example()
    print(f"Stack - Max Flow Before: 0")
    stack_robot.visualize_3d("before_flow.png", subfolder="stack")
    
    flow_value, _ = stack_robot.calculate_max_flow()
    print(f"Stack - Max Flow: {flow_value}")
    stack_robot.visualize_3d("with_flow.png", subfolder="stack")
    
    
    # Create and visualize the simple grid example
    grid_robot = create_simple_grid_example()
    print(f"Simple Grid - Max Flow Before: 0")
    grid_robot.visualize_3d("before_flow.png", subfolder="grid")
    
    # Calculate max flow
    flow_value, _ = grid_robot.calculate_max_flow()
    print(f"Simple Grid - Max Flow: {flow_value}")
    grid_robot.visualize_3d("with_flow.png", subfolder="grid")
    
    # Create and visualize the bridge example
    bridge_robot = create_bridge_example()
    print(f"Bridge - Max Flow Before: 0")
    bridge_robot.visualize_3d("before_flow.png", subfolder="bridge")
    
    # Calculate max flow
    flow_value, _ = bridge_robot.calculate_max_flow()
    print(f"Bridge - Max Flow: {flow_value}")
    bridge_robot.visualize_3d("with_flow.png", subfolder="bridge")
    
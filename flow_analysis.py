import networkx as nx
from networkx.algorithms.flow import edmonds_karp
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime

# Import ModuleType directly from dynamic_bridge_2d
from dynamic_bridge_2d import ModuleType

class FlowAnalyzer:
    """
    Class responsible for analyzing flow and detecting bottlenecks 
    in a modular robot configuration.
    """
    def __init__(self, bridge_system):
        self.bridge_system = bridge_system
        self.flow_graph = nx.DiGraph()
        self.flow_value = 0
        self.flow_dict = {}
        self.bottlenecks = []
        self.path_usage = {}  # Track which paths are being used
    
    def build_flow_graph(self):
        """
        Build a flow graph from the current state of the bridge system.
        Each module becomes a node, and connections between adjacent modules become edges.
        """
        bridge_system = self.bridge_system
        
        # Clear previous graph
        self.flow_graph = nx.DiGraph()
        
        # Add super source and sink
        self.flow_graph.add_node(bridge_system.super_source)
        self.flow_graph.add_node(bridge_system.super_sink)
        
        # Add nodes for all modules except empty spaces
        for module_id, module in bridge_system.modules.items():
            if module.type != ModuleType.EMPTY:
                self.flow_graph.add_node(
                    module_id, 
                    pos=(module.x, module.y),
                    type=module.type
                )
        
        # Connect source modules to super source
        for source in bridge_system.source_modules:
            self.flow_graph.add_edge(
                bridge_system.super_source, 
                source.id,
                capacity=1  # Each source module can only move once
            )
        
        # Connect target positions to super sink
        for target in bridge_system.target_positions:
            self.flow_graph.add_edge(
                target.id,
                bridge_system.super_sink,
                capacity=1  # Each target can only receive one module
            )
        
        # Connect adjacent modules (structure, bridge, unused)
        movable_types = [
            ModuleType.STRUCTURE,
            ModuleType.BRIDGE,
            ModuleType.UNUSED
        ]
        
        # For each module that can be moved to
        for module_id, module in bridge_system.modules.items():
            if module.type in movable_types:
                
                # Get neighbors
                neighbors = bridge_system.get_adjacent_modules(module)
                
                # Connect to each neighbor
                for neighbor in neighbors:
                    if neighbor.type in movable_types or neighbor.type == ModuleType.TARGET:
                        self.flow_graph.add_edge(
                            module_id,
                            neighbor.id,
                            capacity=1  # Each edge can only support one module at a time
                        )
        
        # Connect sources to adjacent structure modules
        for source in bridge_system.source_modules:
            neighbors = bridge_system.get_adjacent_modules(source)
            for neighbor in neighbors:
                if neighbor.type in movable_types:
                    self.flow_graph.add_edge(
                        source.id,
                        neighbor.id,
                        capacity=1
                    )
            
        return self.flow_graph
    
    def calculate_max_flow(self):
        """
        Calculate the maximum flow from super source to super sink
        using the Edmonds-Karp algorithm.
        """
        bridge_system = self.bridge_system
        
        # Build/update the flow graph
        self.build_flow_graph()
        
        # Calculate max flow
        flow_value, self.flow_dict = nx.maximum_flow(
            self.flow_graph, 
            bridge_system.super_source, 
            bridge_system.super_sink, 
            flow_func=edmonds_karp
        )
        
        # Update flow_value
        self.flow_value = flow_value
        
        # Update the graph with flow values
        for u in self.flow_dict:
            for v, flow in self.flow_dict[u].items():
                if self.flow_graph.has_edge(u, v):
                    self.flow_graph[u][v]['flow'] = flow
        
        return self.flow_value, self.flow_dict
    
    def detect_bottlenecks(self, threshold=0.8):
        """
        Detect bottlenecks in the flow network.
        A bottleneck is an edge where the flow equals the capacity (saturated)
        and removing it would significantly reduce the maximum flow.
        
        Args:
            threshold: Minimum flow ratio to consider an edge as potential bottleneck
        
        Returns:
            List of bottleneck edges (u, v, data)
        """
        self.bottlenecks = []
        
        # Ensure we have a flow calculation
        if not hasattr(self, 'flow_dict') or not self.flow_dict:
            self.calculate_max_flow()
        
        # Look for saturated edges
        for u, v, data in self.flow_graph.edges(data=True):
            if 'flow' in data and 'capacity' in data:
                # Skip super source and sink edges
                if u == self.bridge_system.super_source or v == self.bridge_system.super_sink:
                    continue
                    
                # Calculate flow ratio
                flow_ratio = data['flow'] / data['capacity'] if data['capacity'] > 0 else 0
                
                # Check if it's a bottleneck
                if flow_ratio >= threshold:
                    # Store the edge as bottleneck
                    self.bottlenecks.append((u, v, data))
        
        return self.bottlenecks
    
    def find_alternative_paths(self):
        """
        Find alternative paths that could be created to alleviate bottlenecks.
        This involves identifying unused modules that could be repurposed as bridges.
        
        Returns:
            List of potential bridge paths [(module_ids), ...]
        """
        bridge_system = self.bridge_system
        
        # Ensure we have detected bottlenecks
        if not self.bottlenecks:
            self.detect_bottlenecks()
        
        potential_bridge_paths = []
        
        # If there are no bottlenecks, no need for alternative paths
        if not self.bottlenecks:
            return potential_bridge_paths
        
        # Extract bottleneck locations
        bottleneck_locations = []
        for u, v, _ in self.bottlenecks:
            u_module = bridge_system.modules.get(u)
            v_module = bridge_system.modules.get(v)
            
            if u_module and v_module:
                # Calculate the midpoint of the bottleneck
                mid_x = (u_module.x + v_module.x) / 2
                mid_y = (u_module.y + v_module.y) / 2
                bottleneck_locations.append((mid_x, mid_y, u, v))
        
        # For each bottleneck, find potential bypass paths
        for mid_x, mid_y, u, v in bottleneck_locations:
            # Get unused modules in the vicinity of the bottleneck
            unused_modules = [
                m for m in bridge_system.bridge_candidates
                if abs(m.x - mid_x) <= 3 and abs(m.y - mid_y) <= 3
            ]
            
            # Check if we can form a path using these unused modules
            if unused_modules:
                # Find a path from before the bottleneck to after the bottleneck
                u_module = bridge_system.modules.get(u)
                v_module = bridge_system.modules.get(v)
                
                if u_module and v_module:
                    # TODO: Find a path connecting unused modules from near u to near v
                    # For now, just consider groups of nearby unused modules as potential
                    if len(unused_modules) >= 2:
                        potential_bridge_paths.append([m.id for m in unused_modules])
        
        return potential_bridge_paths
    
    def print_flow_results(self):
        """Print the results of the flow calculation"""
        bridge_system = self.bridge_system
        
        print(f"\nMaximum flow: {self.flow_value}")
        print("\nFlow on each edge:")
        for u, v, data in self.flow_graph.edges(data=True):
            if 'flow' in data and data['flow'] > 0:
                u_label = u if u == bridge_system.super_source else f"{u}"
                v_label = v if v == bridge_system.super_sink else f"{v}"
                print(f"{u_label} -> {v_label}: {data['flow']}/{data['capacity']}")
    
    def visualize_flow(self, title="Flow Network", save_path=None):
        """
        Visualize the flow network with flows and capacities.
        """
        bridge_system = self.bridge_system
        fig, ax = plt.subplots(figsize=(15, 10))
        
        # Get positions for nodes
        pos = {}
        for node in self.flow_graph.nodes():
            if node == bridge_system.super_source:
                pos[node] = (-1, bridge_system.height / 2)  # Left of grid
            elif node == bridge_system.super_sink:
                pos[node] = (bridge_system.width + 1, bridge_system.height / 2)  # Right of grid
            else:
                module = bridge_system.modules.get(node)
                if module:
                    # Flip y to match the grid visualization
                    pos[node] = (module.x, bridge_system.height - 1 - module.y)
        
        # Define node colors based on module type
        node_colors = []
        for node in self.flow_graph.nodes():
            if node == bridge_system.super_source:
                node_colors.append('gold')
            elif node == bridge_system.super_sink:
                node_colors.append('purple')
            else:
                module = bridge_system.modules.get(node)
                if module:
                    if module.type == ModuleType.SOURCE:
                        node_colors.append('red')
                    elif module.type == ModuleType.TARGET:
                        node_colors.append('green')
                    elif module.type == ModuleType.STRUCTURE:
                        node_colors.append('lightgray')
                    elif module.type == ModuleType.BRIDGE:
                        node_colors.append('orange')
                    elif module.type == ModuleType.UNUSED:
                        node_colors.append('gray')
                    else:
                        node_colors.append('blue')  # Default
                else:
                    node_colors.append('blue')  # Default
        
        # Draw nodes
        nx.draw_networkx_nodes(self.flow_graph, pos, node_color=node_colors, 
                              node_size=500, alpha=0.8)
        
        # Draw edges with different styles based on flow
        edges_with_flow = [(u, v) for u, v, d in self.flow_graph.edges(data=True) 
                          if 'flow' in d and d['flow'] > 0]
        edges_no_flow = [(u, v) for u, v, d in self.flow_graph.edges(data=True) 
                        if 'flow' not in d or d['flow'] == 0]
        
        # Draw edges without flow (thin gray)
        nx.draw_networkx_edges(self.flow_graph, pos, edgelist=edges_no_flow,
                              width=1, alpha=0.3, edge_color='gray')
        
        # Draw edges with flow (thick red)
        nx.draw_networkx_edges(self.flow_graph, pos, edgelist=edges_with_flow,
                              width=2, alpha=1.0, edge_color='red')
        
        # Draw bottlenecks (even thicker, different color)
        bottleneck_edges = [(u, v) for u, v, _ in self.bottlenecks]
        nx.draw_networkx_edges(self.flow_graph, pos, edgelist=bottleneck_edges,
                              width=3, alpha=1.0, edge_color='darkred')
        
        # Draw labels
        nx.draw_networkx_labels(self.flow_graph, pos, font_size=10, font_weight='bold')
        
        # Draw edge labels (flow/capacity)
        edge_labels = {}
        for u, v, data in self.flow_graph.edges(data=True):
            if 'flow' in data and 'capacity' in data:
                edge_labels[(u, v)] = f"{data['flow']}/{data['capacity']}"
        
        nx.draw_networkx_edge_labels(self.flow_graph, pos, edge_labels=edge_labels,
                                    font_size=8)
        
        # Title and layout
        plt.title(title)
        plt.axis('off')
        plt.tight_layout()
        
        # Save if path is provided
        if save_path:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show() 
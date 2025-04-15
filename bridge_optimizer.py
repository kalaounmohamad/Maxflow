import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import copy
from dynamic_bridge_2d import ModuleType

class VirtualGridAnalyzer:
    """
    Class for analyzing a virtual full grid to determine optimal bridge placements.
    This class works with the DynamicBridgeSystem to suggest ideal bridge locations.
    """
    def __init__(self, bridge_system):
        """
        Initialize the virtual grid analyzer with a reference to the actual bridge system.
        
        Args:
            bridge_system: The DynamicBridgeSystem instance to analyze
        """
        self.bridge_system = bridge_system
        self.width = bridge_system.width
        self.height = bridge_system.height
        self.virtual_graph = nx.DiGraph()
        self.optimal_paths = []
        self.bridge_candidates = []
        self.bottlenecks = []
        
        # Copy module references but use a virtual grid
        self.grid = copy.deepcopy(bridge_system.grid)
        
        # Initially mark all structure modules as unused for analysis
        self.mark_all_structure_as_unused()
        
    def mark_all_structure_as_unused(self):
        """
        Mark all structure modules as unused initially.
        This allows us to identify which ones are actually needed in flow paths.
        """
        print("Marking all structure modules as unused for analysis...")
        for module_id, module in self.bridge_system.modules.items():
            if module.type == ModuleType.STRUCTURE:
                module.type = ModuleType.UNUSED
                print(f"Marked module {module_id} at ({module.x}, {module.y}) as unused")

    def create_virtual_full_grid(self):
        """
        Create a virtual grid where all empty spaces are treated as walkable with capacity 1.
        """
        print("Creating virtual full grid for analysis...")
        
        # Clear the virtual graph and create a fresh one
        self.virtual_graph = nx.DiGraph()
        
        # Add nodes for all positions (including empty spaces)
        for y in range(self.height):
            for x in range(self.width):
                # Get the actual module at this position
                actual_module = self.bridge_system.grid[y][x]
                node_id = f"V_{x}_{y}"
                
                # Add the node with its position and type info
                if actual_module:
                    self.virtual_graph.add_node(
                        node_id, 
                        pos=(x, y), 
                        type=actual_module.type,
                        is_virtual=(actual_module.type.name == "EMPTY")
                    )
        
        # Add super source and sink
        self.virtual_graph.add_node(
            "Super_S", 
            pos=(-1, self.height//2), 
            type="super_source",
            is_virtual=False
        )
        
        self.virtual_graph.add_node(
            "Super_T", 
            pos=(self.width, self.height//2), 
            type="super_sink",
            is_virtual=False
        )
        
        # Connect source modules to super source
        for source in self.bridge_system.source_modules:
            source_id = f"V_{source.x}_{source.y}"
            self.virtual_graph.add_edge("Super_S", source_id, capacity=1)
        
        # Connect target positions to super sink
        for target in self.bridge_system.target_positions:
            target_id = f"V_{target.x}_{target.y}"
            self.virtual_graph.add_edge(target_id, "Super_T", capacity=1)
        
        # Connect all adjacent cells in the grid (including virtual ones)
        for y in range(self.height):
            for x in range(self.width):
                current_id = f"V_{x}_{y}"
                
                # Define offsets for adjacent cells (8-connectivity)
                offsets = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                if self.bridge_system.allow_diagonal_moves:
                    offsets.extend([(1, 1), (1, -1), (-1, -1), (-1, 1)])
                
                for dx, dy in offsets:
                    adj_x, adj_y = x + dx, y + dy
                    
                    # Check if position is within grid bounds
                    if 0 <= adj_x < self.width and 0 <= adj_y < self.height:
                        adj_id = f"V_{adj_x}_{adj_y}"
                        # Add bidirectional edges with capacity 1
                        self.virtual_graph.add_edge(current_id, adj_id, capacity=1)
                        self.virtual_graph.add_edge(adj_id, current_id, capacity=1)
        
        print(f"Virtual grid created with {len(self.virtual_graph.nodes)} nodes and {len(self.virtual_graph.edges)} edges")
        return self.virtual_graph
    
    def calculate_virtual_max_flow(self):
        """
        Calculate maximum flow on the virtual full grid.
        
        Returns:
            tuple: (flow_value, flow_dict)
        """
        # Ensure virtual graph is built
        if not hasattr(self.virtual_graph, 'edges') or len(self.virtual_graph.edges) == 0:
            self.create_virtual_full_grid()
        
        print("Calculating maximum flow on virtual full grid...")
        
        # Calculate maximum flow
        flow_value, flow_dict = nx.maximum_flow(
            self.virtual_graph, "Super_S", "Super_T", 
            flow_func=nx.algorithms.flow.edmonds_karp
        )
        
        # Update graph with flow values
        for u in flow_dict:
            for v, flow in flow_dict[u].items():
                self.virtual_graph[u][v]['flow'] = flow
        
        print(f"Virtual maximum flow value: {flow_value}")
        return flow_value, flow_dict
    
    def identify_optimal_paths(self):
        """
        Identify optimal paths from sources to targets based on the flow result.
        
        Returns:
            list: List of paths (each path is a list of node IDs)
        """
        # Ensure flow has been calculated
        if not any('flow' in data for _, _, data in self.virtual_graph.edges(data=True)):
            self.calculate_virtual_max_flow()
        
        print("Identifying optimal paths in virtual grid...")
        
        # Create a subgraph containing only edges with positive flow
        flow_edges = [(u, v) for u, v, data in self.virtual_graph.edges(data=True) 
                     if data.get('flow', 0) > 0]
        flow_graph = self.virtual_graph.edge_subgraph(flow_edges).copy()
        
        # Find paths from each source to each target
        self.optimal_paths = []
        source_ids = [f"V_{s.x}_{s.y}" for s in self.bridge_system.source_modules]
        target_ids = [f"V_{t.x}_{t.y}" for t in self.bridge_system.target_positions]
        
        for source_id in source_ids:
            for target_id in target_ids:
                # Check if path exists
                if nx.has_path(flow_graph, source_id, target_id):
                    # Find shortest path
                    path = nx.shortest_path(flow_graph, source_id, target_id)
                    self.optimal_paths.append(path)
                    print(f"Found optimal path from {source_id} to {target_id} with {len(path)} nodes")
        
        print(f"Identified {len(self.optimal_paths)} optimal paths in the virtual grid")
        
        # Now mark modules that are used in these paths as "structure" (not unused)
        self.mark_used_modules_in_flow()
        
        return self.optimal_paths
    
    def mark_used_modules_in_flow(self):
        """
        Mark modules that are used in optimal flow paths as "structure" instead of "unused".
        """
        print("Marking modules used in flow paths...")
        
        # Create a set to track which positions are used in the flow
        used_positions = set()
        
        # Extract positions from all optimal paths
        for path in self.optimal_paths:
            # Skip super source and super sink
            for node in path:
                if node != "Super_S" and node != "Super_T" and node.startswith("V_"):
                    parts = node.split("_")
                    x, y = int(parts[1]), int(parts[2])
                    used_positions.add((x, y))
        
        # Mark used modules as structure
        structure_count = 0
        for module_id, module in self.bridge_system.modules.items():
            # Check if this module's position is in a flow path
            if module.type == ModuleType.UNUSED and (module.x, module.y) in used_positions:
                module.type = ModuleType.STRUCTURE
                structure_count += 1
                print(f"Module {module_id} at ({module.x}, {module.y}) is used in flow path - marking as STRUCTURE")
        
        print(f"Marked {structure_count} modules as STRUCTURE (used in flow paths)")
    
    def identify_bridge_candidates(self):
        """
        Identify potential bridge locations based on optimal paths through empty spaces.
        
        Returns:
            list: List of bridge candidates (each candidate is a list of positions)
        """
        # Ensure optimal paths have been identified
        if not self.optimal_paths:
            self.identify_optimal_paths()
        
        print("Identifying bridge candidates...")
        self.bridge_candidates = []
        
        # Find bridge candidates in each optimal path
        for path in self.optimal_paths:
            # Skip super source and super sink
            path = [node for node in path if node != "Super_S" and node != "Super_T"]
            
            bridge_segments = []
            current_segment = []
            
            # Parse each node ID to get coordinates
            for node_id in path:
                if node_id.startswith("V_"):
                    parts = node_id.split("_")
                    x, y = int(parts[1]), int(parts[2])
                    
                    # Check if the real grid has an empty space here
                    real_module = self.bridge_system.grid[y][x]
                    if real_module.type.name == "EMPTY":
                        current_segment.append((x, y))
                    else:
                        # End of empty segment
                        if current_segment:
                            bridge_segments.append(current_segment)
                            current_segment = []
            
            # Add the last segment if it exists
            if current_segment:
                bridge_segments.append(current_segment)
            
            # Filter segments to include only those that would connect left and right structure
            valid_segments = []
            for segment in bridge_segments:
                if segment and self.is_segment_connecting_structures(segment):
                    valid_segments.append(segment)
                    print(f"Found valid bridge candidate with {len(segment)} positions: {segment}")
            
            # Add valid segments as bridge candidates
            for segment in valid_segments:
                self.bridge_candidates.append(segment)
        
        # Merge overlapping bridge candidates
        self.bridge_candidates = self.merge_overlapping_candidates(self.bridge_candidates)
        
        print(f"Identified {len(self.bridge_candidates)} bridge candidates after merging")
        return self.bridge_candidates
    
    def is_segment_connecting_structures(self, segment):
        """
        Determine if a bridge segment would actually connect the left and right structure blocks.
        
        Args:
            segment: List of (x, y) positions representing a potential bridge
            
        Returns:
            bool: True if the segment would connect structures on both sides
        """
        if not segment:
            return False
        
        # Get the leftmost and rightmost positions
        min_x = min(x for x, _ in segment)
        max_x = max(x for x, _ in segment)
        
        # The segment should span across the gap (from columns 4-5 to columns 7-8)
        if not (min_x <= 5 and max_x >= 7):
            return False
        
        # Check if there are structure modules adjacent to both ends
        left_adjacent = False
        right_adjacent = False
        
        # For each position in the segment, check if it's adjacent to a structure
        for x, y in segment:
            # Check all adjacent positions
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]:
                adj_x, adj_y = x + dx, y + dy
                
                if 0 <= adj_x < self.width and 0 <= adj_y < self.height:
                    adj_module = self.bridge_system.grid[adj_y][adj_x]
                    
                    if adj_module.type == ModuleType.STRUCTURE:
                        if adj_x < 5:  # Left side structure
                            left_adjacent = True
                        elif adj_x >= 8:  # Right side structure
                            right_adjacent = True
        
        return left_adjacent and right_adjacent
    
    def merge_overlapping_candidates(self, candidates):
        """
        Merge bridge candidates that overlap or are adjacent.
        
        Args:
            candidates: List of bridge candidates (each candidate is a list of positions)
            
        Returns:
            list: Merged bridge candidates
        """
        if not candidates:
            return []
        
        # Convert positions to sets for easier manipulation
        candidate_sets = [set(candidate) for candidate in candidates]
        merged = []
        
        while candidate_sets:
            current = candidate_sets.pop(0)
            merged_set = False
            
            for i, other in enumerate(candidate_sets):
                # Check if any position in current is adjacent to any position in other
                for x1, y1 in current:
                    for x2, y2 in other:
                        # Check if positions are adjacent (including diagonals)
                        if abs(x1 - x2) <= 1 and abs(y1 - y2) <= 1:
                            # Merge sets and replace other with merged set
                            candidate_sets[i] = current.union(other)
                            merged_set = True
                            break
                    if merged_set:
                        break
                
                if merged_set:
                    break
            
            # If current wasn't merged with any other set, add it to the result
            if not merged_set:
                merged.append(list(current))
        
        return merged
    
    def identify_bottlenecks(self):
        """
        Identify bottlenecks in the real flow network.
        
        Returns:
            list: List of bottleneck module IDs
        """
        print("Identifying bottlenecks in the real flow network...")
        self.bottlenecks = self.bridge_system.detect_bottlenecks()
        return self.bottlenecks
    
    def prioritize_bridge_candidates(self):
        """
        Prioritize bridge candidates based on proximity to existing rows with bridges and flow paths.
        
        Returns:
            list: Prioritized bridge candidates
        """
        # Ensure we have bridge candidates
        if not self.bridge_candidates:
            self.identify_bridge_candidates()
        
        print("Prioritizing bridge candidates...")
        
        # For each candidate, calculate a priority score
        # Lower scores are better (will be sorted first)
        candidate_scores = []
        
        # Find the rows that currently have bridges (row 4 in the original setup)
        existing_bridge_rows = set()
        for module_id, module in self.bridge_system.modules.items():
            if module.type == ModuleType.STRUCTURE and 5 <= module.x <= 7:
                existing_bridge_rows.add(module.y)
        
        print(f"Existing bridge rows: {existing_bridge_rows}")
        
        for candidate in self.bridge_candidates:
            # Group positions by row
            rows = set(y for _, y in candidate)
            
            # Calculate average row of this candidate
            avg_row = sum(y for _, y in candidate) / len(candidate)
            
            # Calculate minimum distance to any existing bridge row
            min_dist_to_bridge = min(abs(avg_row - bridge_row) for bridge_row in existing_bridge_rows) if existing_bridge_rows else 0
            
            # Prefer rows that are at least 1-2 rows away from existing bridges
            # (0 = good, higher = worse)
            if min_dist_to_bridge == 0:
                row_score = 100  # Heavily penalize building in the same row
            elif min_dist_to_bridge <= 2:
                row_score = 0  # Good - near but not on existing bridges
            else:
                row_score = min_dist_to_bridge  # Further is increasingly worse
            
            # Prefer candidates that are in rows 3 or 5 (above and below existing bridge)
            target_rows = {3, 5}
            row_match_score = 0 if any(y in target_rows for _, y in candidate) else 10
            
            # Calculate the average x-coordinate (prefer candidates in the middle)
            avg_x = sum(x for x, _ in candidate) / len(candidate)
            middle_score = abs(avg_x - self.width / 2)
            
            # Calculate total score (lower is better)
            total_score = row_score + row_match_score + middle_score
            
            candidate_scores.append((candidate, total_score))
            print(f"Candidate at rows {rows} has score {total_score}")
        
        # Sort by score (ascending)
        candidate_scores.sort(key=lambda x: x[1])
        
        # Extract just the candidates
        sorted_candidates = [candidate for candidate, _ in candidate_scores]
        
        print(f"Prioritized {len(sorted_candidates)} bridge candidates")
        return sorted_candidates
    
    def visualize_virtual_flow(self, title="Virtual Flow Visualization", show=True, save_path=None):
        """
        Visualize the virtual flow graph with flow values.
        
        Args:
            title: Title for the visualization
            show: Whether to display the plot
            save_path: Path to save the visualization image
        """
        if not hasattr(self.virtual_graph, 'edges') or len(self.virtual_graph.edges) == 0:
            print("Virtual flow graph not built.")
            return
        
        fig, ax = plt.subplots(figsize=(15, 8))
        
        # Get positions from the grid
        pos = {}
        for node in self.virtual_graph.nodes():
            if node == "Super_S":
                pos[node] = (-1, self.height//2)
            elif node == "Super_T":
                pos[node] = (self.width, self.height//2)
            elif node.startswith("V_"):
                parts = node.split("_")
                x, y = int(parts[1]), int(parts[2])
                pos[node] = (x, self.height - 1 - y)
        
        # Draw nodes with different colors based on type
        node_colors = []
        for node in self.virtual_graph.nodes():
            if node == "Super_S":
                node_colors.append('gold')
            elif node == "Super_T":
                node_colors.append('purple')
            elif node.startswith("V_"):
                parts = node.split("_")
                x, y = int(parts[1]), int(parts[2])
                module = self.bridge_system.grid[y][x]
                
                if module.type.name == "SOURCE":
                    node_colors.append('red')
                elif module.type.name == "TARGET":
                    node_colors.append('green')
                elif module.type.name == "STRUCTURE":
                    node_colors.append('black')
                elif module.type.name == "UNUSED":
                    node_colors.append('gray')
                elif module.type.name == "BRIDGE":
                    node_colors.append('orange')
                elif module.type.name == "EMPTY":
                    node_colors.append('lightblue')  # Virtual nodes (empty spaces)
                else:
                    node_colors.append('white')
        
        # Draw nodes
        nx.draw_networkx_nodes(self.virtual_graph, pos, node_color=node_colors, 
                              node_size=200, alpha=0.8)
        
        # Draw edges with flow values
        edges_with_flow = [(u, v) for u, v, data in self.virtual_graph.edges(data=True) 
                          if data.get('flow', 0) > 0]
        
        # Draw edges with flow (thicker, red)
        if edges_with_flow:
            edge_width = [self.virtual_graph[u][v].get('flow', 0) * 3 for u, v in edges_with_flow]
            nx.draw_networkx_edges(self.virtual_graph, pos, edgelist=edges_with_flow, 
                                  width=edge_width, edge_color='red', alpha=0.7)
        
        # Draw edges without flow (thin, gray)
        edges_no_flow = [(u, v) for u, v, data in self.virtual_graph.edges(data=True) 
                        if data.get('flow', 0) == 0]
        if edges_no_flow:
            nx.draw_networkx_edges(self.virtual_graph, pos, edgelist=edges_no_flow, 
                                  width=0.5, edge_color='gray', alpha=0.1, style='dashed')
        
        # Add edge labels (flow/capacity) for edges with flow
        edge_labels = {(u, v): f"{data.get('flow', 0)}" 
                      for u, v, data in self.virtual_graph.edges(data=True)
                      if data.get('flow', 0) > 0}
        nx.draw_networkx_edge_labels(self.virtual_graph, pos, edge_labels=edge_labels, font_size=8)
        
        # Add legend
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                      markersize=10, label='Source'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                      markersize=10, label='Target'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='black', 
                      markersize=10, label='Structure'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                      markersize=10, label='Virtual (Empty Space)'),
            plt.Line2D([0], [0], color='red', lw=2, label='Flow Path')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
        # Highlight bridge candidates if available
        if self.bridge_candidates:
            for candidate in self.bridge_candidates:
                for x, y in candidate:
                    circle = plt.Circle(
                        (x, self.height - 1 - y), 
                        0.4, 
                        fill=True,
                        edgecolor='blue', 
                        facecolor='lightblue',
                        alpha=0.5,
                        linewidth=2
                    )
                    ax.add_patch(circle)
        
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
    
    def visualize_bridge_candidates(self, title="Bridge Candidates", show=True, save_path=None):
        """
        Visualize the real grid with bridge candidates highlighted.
        
        Args:
            title: Title for the visualization
            show: Whether to display the plot
            save_path: Path to save the visualization image
        """
        # Copy the grid visualization function but add highlights for bridge candidates
        fig, ax = plt.subplots(figsize=(15, 8))
        
        # Colors for different module types (fill and outline)
        fill_colors = {
            "SOURCE": 'red',
            "TARGET": 'white',
            "STRUCTURE": 'white',
            "UNUSED": 'white',
            "BRIDGE": 'orange',
            "EMPTY": 'white'
        }
        
        edge_colors = {
            "SOURCE": 'black',
            "TARGET": 'green',
            "STRUCTURE": 'black',
            "UNUSED": 'gray',
            "BRIDGE": 'orange',
            "EMPTY": 'none'
        }
        
        # Flag for whether to fill the circle
        fill_module = {
            "SOURCE": True,
            "TARGET": False,
            "STRUCTURE": False,
            "UNUSED": False,
            "BRIDGE": True,
            "EMPTY": False
        }
        
        # Create a set of bridge candidate positions for fast lookup
        candidate_positions = set()
        for candidate in self.bridge_candidates:
            for x, y in candidate:
                candidate_positions.add((x, y))
        
        # Plotting each module
        for y in range(self.height):
            for x in range(self.width):
                module = self.bridge_system.grid[y][x]
                if module:
                    if module.type.name == "EMPTY":
                        # Highlight empty spaces that are bridge candidates
                        if (x, y) in candidate_positions:
                            circle = plt.Circle(
                                (x, self.height - 1 - y), 
                                0.4, 
                                fill=True,
                                edgecolor='blue', 
                                facecolor='lightblue',
                                alpha=0.5,
                                linewidth=2
                            )
                            ax.add_patch(circle)
                    else:
                        # Base circle for the module
                        circle = plt.Circle(
                            (x, self.height - 1 - y), 
                            0.4, 
                            fill=fill_module[module.type.name],
                            edgecolor=edge_colors[module.type.name], 
                            facecolor=fill_colors[module.type.name],
                            linewidth=2
                        )
                        ax.add_patch(circle)
                        
                        # Add a label for bridge modules
                        if module.type.name == "BRIDGE":
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
                      markeredgecolor='orange', markersize=15, label='Bridge'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                      markeredgecolor='blue', markersize=15, label='Bridge Candidate')
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
    
    def find_closest_unused_modules(self, bridge_positions):
        """
        Find the closest unused modules for the bridge positions.
        
        Args:
            bridge_positions: List of (x, y) positions for the bridge
            
        Returns:
            dict: Mapping from bridge position to closest unused module
        """
        # Get all unused modules (these were not part of optimal flow paths)
        unused_modules = [m for m in self.bridge_system.modules.values() 
                         if m.type == ModuleType.UNUSED]
        
        print(f"Found {len(unused_modules)} unused modules available for bridge construction")
        
        if not unused_modules:
            print("Warning: No unused modules available for bridge construction!")
            return {}
        
        # Map each bridge position to the closest unused module
        position_to_module = {}
        assigned_modules = set()
        
        # Sort bridge positions by proximity to any unused module (to handle multiple bridge positions)
        sorted_positions = []
        for pos in bridge_positions:
            min_distance = float('inf')
            for module in unused_modules:
                distance = abs(pos[0] - module.x) + abs(pos[1] - module.y)
                min_distance = min(min_distance, distance)
            sorted_positions.append((pos, min_distance))
        
        sorted_positions.sort(key=lambda x: x[1])
        bridge_positions = [pos for pos, _ in sorted_positions]
        
        # Assign the closest unused module to each bridge position
        for pos in bridge_positions:
            closest_module = None
            min_distance = float('inf')
            
            for module in unused_modules:
                if module.id in assigned_modules:
                    continue
                
                distance = abs(pos[0] - module.x) + abs(pos[1] - module.y)
                if distance < min_distance:
                    min_distance = distance
                    closest_module = module
            
            if closest_module:
                position_to_module[pos] = closest_module
                assigned_modules.add(closest_module.id)
                print(f"Assigned module {closest_module.id} at ({closest_module.x}, {closest_module.y}) to bridge position {pos}")
        
        # If there are more bridge positions than unused modules, some positions will remain unmapped
        unused_positions = [pos for pos in bridge_positions if pos not in position_to_module]
        if unused_positions:
            print(f"Warning: Not enough unused modules for {len(unused_positions)} bridge positions")
        
        return position_to_module
    
    def simulate_bridge_construction(self, bridge_positions):
        """
        Simulate bridge construction to verify flow improvement.
        
        Args:
            bridge_positions: List of (x, y) positions for the bridge
            
        Returns:
            tuple: (success, flow_improvement, position_to_module)
        """
        print(f"Simulating bridge construction for {len(bridge_positions)} positions...")
        
        # Create a copy of the bridge system for simulation
        sim_system = copy.deepcopy(self.bridge_system)
        
        # Calculate initial flow before any changes
        original_flow, _ = sim_system.calculate_max_flow()
        print(f"Initial flow value in simulation: {original_flow}")
        
        # Find the closest unused modules for each bridge position
        position_to_module = self.find_closest_unused_modules(bridge_positions)
        
        # Track which positions were successfully filled
        filled_positions = []
        
        # Simulate moving each module to its bridge position
        for pos, module in position_to_module.items():
            x, y = pos
            print(f"Simulating move of module {module.id} to position ({x}, {y})...")
            
            # Check if removing this module would break connectivity
            # Make a temporary copy to check the impact
            temp_system = copy.deepcopy(sim_system)
            original_module = temp_system.modules[module.id]
            
            # Temporarily mark the module as empty to check if it breaks flow
            original_type = original_module.type
            original_module.type = ModuleType.EMPTY
            
            # Rebuild flow graph and check if flow decreased
            temp_system.build_flow_graph()
            temp_flow, _ = temp_system.calculate_max_flow()
            
            if temp_flow < original_flow:
                print(f"Warning: Moving module {module.id} would disrupt existing flow. Skipping.")
                continue
            
            # Try to move the module in the simulation
            if sim_system.move_module(module, x, y):
                filled_positions.append(pos)
            else:
                print(f"Warning: Failed to move module {module.id} to position ({x}, {y}) in simulation")
        
        # Rebuild flow graph and calculate new flow
        sim_system.build_flow_graph()
        new_flow, _ = sim_system.calculate_max_flow()
        
        # Calculate improvement
        flow_improvement = new_flow - original_flow
        
        print(f"Simulated bridge construction results:")
        print(f"  - Positions filled: {len(filled_positions)} out of {len(bridge_positions)}")
        print(f"  - Original flow: {original_flow}")
        print(f"  - New flow: {new_flow}")
        print(f"  - Flow improvement: {flow_improvement}")
        
        # Success if any positions were filled and there's positive flow improvement
        success = len(filled_positions) > 0 and flow_improvement >= 0
        
        return success, flow_improvement, position_to_module
    
    def build_optimal_bridges(self):
        """
        Build optimal bridges based on virtual flow analysis.
        
        Returns:
            list: List of bridges that were built
        """
        # Ensure we have prioritized bridge candidates
        prioritized_candidates = self.prioritize_bridge_candidates()
        if not prioritized_candidates:
            print("No bridge candidates identified.")
            return []
        
        print(f"Building optimal bridges from {len(prioritized_candidates)} candidates...")
        
        # Track the bridges we build
        built_bridges = []
        total_flow_improvement = 0
        
        # Try each candidate in order of priority
        for bridge_positions in prioritized_candidates:
            print(f"\nEvaluating bridge candidate with {len(bridge_positions)} positions...")
            
            # Simulate bridge construction to verify improvements
            success, flow_improvement, position_to_module = self.simulate_bridge_construction(bridge_positions)
            
            if success:
                print(f"Bridge candidate would improve flow by {flow_improvement}. Building...")
                
                # Actually build the bridge
                bridge_modules = []
                
                for pos, module in position_to_module.items():
                    x, y = pos
                    print(f"Moving module {module.id} from ({module.x}, {module.y}) to ({x}, {y})...")
                    
                    if self.bridge_system.move_module(module, x, y):
                        bridge_modules.append(module)
                        # Ensure it's marked as bridge type
                        module.type = ModuleType.BRIDGE
                    else:
                        print(f"Warning: Failed to move module {module.id} to position ({x}, {y})")
                
                # If we successfully built at least part of the bridge
                if bridge_modules:
                    built_bridges.append(bridge_modules)
                    total_flow_improvement += flow_improvement
                    
                    # Visualize the current state
                    self.bridge_system.visualize_grid(
                        f"After building bridge {len(built_bridges)}", 
                        save_path=f"images_2d/simulation/bridge_{len(built_bridges)}_built.png",
                        show=False
                    )
                    
                    # Rebuild the flow graph to include the new bridge
                    self.bridge_system.build_flow_graph()
                    
                    # Calculate the new flow
                    flow_value, _ = self.bridge_system.calculate_max_flow()
                    print(f"Flow value after bridge {len(built_bridges)}: {flow_value}")
                    
                    # Visualize the flow with the new bridge
                    self.bridge_system.visualize_flow(
                        f"Flow after bridge {len(built_bridges)}", 
                        save_path=f"images_2d/simulation/flow_after_bridge_{len(built_bridges)}.png",
                        show=False
                    )
                    
                    # Update our bridge candidates and bottlenecks
                    self.create_virtual_full_grid()
                    self.calculate_virtual_max_flow()
                    self.identify_bridge_candidates()
                    self.identify_bottlenecks()
                    prioritized_candidates = self.prioritize_bridge_candidates()
            else:
                print(f"Bridge candidate would not improve flow. Skipping...")
        
        print(f"\nBridge construction complete.")
        print(f"Built {len(built_bridges)} bridges with total flow improvement of {total_flow_improvement}")
        
        return built_bridges


# Example usage
if __name__ == "__main__":
    # Import the DynamicBridgeSystem
    from dynamic_bridge_2d import DynamicBridgeSystem
    
    # Create a 13x9 grid for our example
    system = DynamicBridgeSystem(13, 9)
    
    # Initialize with the example configuration
    system.initialize_from_example()
    
    # Visualize the initial state
    system.visualize_grid("Initial Configuration", 
                       save_path="images_2d/simulation/01_initial_configuration.png")
    
    # Calculate initial flow
    flow_value, _ = system.calculate_max_flow()
    print(f"Initial flow value: {flow_value}")
    
    # Visualize initial flow
    system.visualize_flow("Initial Flow", 
                       save_path="images_2d/simulation/02_initial_flow.png")
    
    # Create a virtual grid analyzer
    analyzer = VirtualGridAnalyzer(system)
    
    # Create virtual full grid
    analyzer.create_virtual_full_grid()
    
    # Calculate virtual max flow
    analyzer.calculate_virtual_max_flow()
    
    # Identify optimal paths (this will also mark used modules)
    analyzer.identify_optimal_paths()
    
    # Visualize virtual flow
    analyzer.visualize_virtual_flow("Virtual Flow on Full Grid", 
                                  save_path="images_2d/simulation/03_virtual_flow.png")
    
    # At this point we've marked structure modules used in flow paths
    # Let's visualize the unused modules
    system.visualize_grid("Unused Modules After Flow Analysis", 
                       save_path="images_2d/simulation/03_unused_modules_identified.png")
    
    # Identify bridge candidates
    analyzer.identify_bridge_candidates()
    
    # Visualize bridge candidates
    analyzer.visualize_bridge_candidates("Bridge Candidates", 
                                      save_path="images_2d/simulation/04_bridge_candidates.png")
    
    # Identify bottlenecks
    analyzer.identify_bottlenecks()
    
    # Prioritize bridge candidates
    prioritized_candidates = analyzer.prioritize_bridge_candidates()
    
    # Build optimal bridges
    built_bridges = analyzer.build_optimal_bridges()
    
    # Visualize final state
    system.visualize_grid("Final Configuration", 
                       save_path="images_2d/simulation/06_final_configuration.png")
    
    # Calculate final flow
    flow_value, _ = system.calculate_max_flow()
    print(f"Final flow value: {flow_value}")
    
    # Visualize final flow
    system.visualize_flow("Final Flow", 
                       save_path="images_2d/simulation/07_final_flow.png")
    
    print("\nBridge optimization complete.")
    print(f"Built {len(built_bridges)} optimal bridges.")
    print("Press Enter to continue...")
    input() 
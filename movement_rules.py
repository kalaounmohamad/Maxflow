import networkx as nx
from collections import deque
from enum import Enum
import copy
import numpy as np

# Import ModuleType directly from dynamic_bridge_2d
from dynamic_bridge_2d import ModuleType

class MovementPlanner:
    """
    Class responsible for planning module movements based on flow analysis results.
    Handles path finding, movement validation, and execution.
    """
    def __init__(self, bridge_system, flow_analyzer):
        self.bridge_system = bridge_system
        self.flow_analyzer = flow_analyzer
        self.planned_movements = []  # List of planned movements [(module, path), ...]
        self.movement_graph = nx.DiGraph()  # Graph for path planning
    
    def build_movement_graph(self):
        """
        Build a graph for planning module movements.
        This is different from the flow graph as it only includes physical 
        positions where modules can move.
        """
        bridge_system = self.bridge_system
        
        # Clear previous graph
        self.movement_graph = nx.DiGraph()
        
        # Add nodes for all physical modules (except sources, which will move)
        movable_types = [
            ModuleType.STRUCTURE,
            ModuleType.BRIDGE,
            ModuleType.UNUSED,
            ModuleType.TARGET
        ]
        
        for module_id, module in bridge_system.modules.items():
            if module.type in movable_types:
                # Each node is a position where a module can move to
                self.movement_graph.add_node(
                    module_id, 
                    pos=(module.x, module.y),
                    type=module.type,
                    occupied=module.occupied
                )
        
        # Connect adjacent nodes (for movement paths)
        for module_id, module in bridge_system.modules.items():
            if module.type in movable_types:
                # Get neighbors
                neighbors = bridge_system.get_adjacent_modules(module)
                
                # Connect to each neighbor
                for neighbor in neighbors:
                    if neighbor.type in movable_types:
                        # Add edge between adjacent modules
                        # Weight can represent difficulty of movement, default to 1
                        self.movement_graph.add_edge(module_id, neighbor.id, weight=1)
        
        return self.movement_graph
    
    def find_path(self, start_module, end_module):
        """
        Find a path from start_module to end_module using the movement graph.
        
        Args:
            start_module: Starting module
            end_module: Target module
            
        Returns:
            List of module IDs representing the path, or None if no path exists
        """
        # Ensure movement graph is built
        self.build_movement_graph()
        
        # Check if start and end are in the graph
        if start_module.id not in self.movement_graph or end_module.id not in self.movement_graph:
            return None
        
        # Find shortest path
        try:
            path = nx.shortest_path(
                self.movement_graph, 
                source=start_module.id, 
                target=end_module.id,
                weight='weight'
            )
            return path
        except nx.NetworkXNoPath:
            return None
    
    def plan_source_movements(self):
        """
        Plan movements for source modules based on flow analysis.
        This identifies which source modules should move to which targets.
        
        Returns:
            List of (source_module, target_module) pairs
        """
        bridge_system = self.bridge_system
        flow_analyzer = self.flow_analyzer
        
        # Ensure we have flow data
        if not flow_analyzer.flow_dict:
            flow_analyzer.calculate_max_flow()
        
        source_target_pairs = []
        
        # For each source module, find where it flows to
        for source_module in bridge_system.source_modules:
            # Check if there's flow from this source
            source_flows = flow_analyzer.flow_dict.get(source_module.id, {})
            
            # Look for outgoing flows
            for next_id, flow in source_flows.items():
                if flow > 0:
                    # There's flow from this source to this next module
                    
                    # Follow the flow from source to target
                    path = self.follow_flow_path(source_module.id)
                    
                    if path:
                        # Get the target module (the last module in the path)
                        target_id = path[-1]
                        target_module = bridge_system.modules.get(target_id)
                        
                        if target_module and target_module.type == ModuleType.TARGET:
                            # Add source-target pair
                            source_target_pairs.append((source_module, target_module, path))
        
        return source_target_pairs
    
    def follow_flow_path(self, start_id):
        """
        Follow a flow path from a starting module to a target.
        
        Args:
            start_id: ID of the starting module
            
        Returns:
            List of module IDs in the path, or None if no path exists
        """
        bridge_system = self.bridge_system
        flow_analyzer = self.flow_analyzer
        
        # Start with empty path
        path = [start_id]
        current_id = start_id
        
        # Keep following flow until we reach a target or dead end
        while True:
            # Get outgoing flows from current module
            outflows = flow_analyzer.flow_dict.get(current_id, {})
            
            # Find the next module with positive flow
            next_id = None
            for mod_id, flow in outflows.items():
                if flow > 0 and mod_id != bridge_system.super_sink:
                    next_id = mod_id
                    break
            
            # If no next module with flow, check if we're at a target
            if not next_id:
                current_module = bridge_system.modules.get(current_id)
                
                # If we're at a target connected to super sink, we're done
                if current_module and current_module.type == ModuleType.TARGET:
                    return path
                
                # Otherwise, dead end
                return None
            
            # Check for loops
            if next_id in path:
                return None
            
            # Add to path and continue
            path.append(next_id)
            current_id = next_id
    
    def plan_bridge_formations(self):
        """
        Plan the formation of new bridges based on bottleneck detection.
        Identifies which unused modules should move to form bridges.
        
        Returns:
            List of bridge formation plans
            [(bridge_id, [module_ids_to_form_bridge]), ...]
        """
        bridge_system = self.bridge_system
        flow_analyzer = self.flow_analyzer
        
        # Detect bottlenecks
        bottlenecks = flow_analyzer.detect_bottlenecks()
        
        # If no bottlenecks, no bridges needed
        if not bottlenecks:
            return []
        
        # Find potential bridge paths
        potential_paths = flow_analyzer.find_alternative_paths()
        
        # For each potential path, plan the bridge formation
        bridge_plans = []
        for i, path_modules in enumerate(potential_paths):
            # Generate a unique ID for this bridge
            bridge_id = f"Bridge_{i}"
            
            # Collect modules that would form this bridge
            modules_to_move = []
            for module_id in path_modules:
                module = bridge_system.modules.get(module_id)
                if module and module.type == ModuleType.UNUSED:
                    modules_to_move.append(module_id)
            
            # Add the plan if we have modules to move
            if modules_to_move:
                bridge_plans.append((bridge_id, modules_to_move))
        
        return bridge_plans
    
    def is_valid_move(self, module, new_x, new_y):
        """
        Check if moving a module to a new position is valid.
        
        Args:
            module: The module to move
            new_x, new_y: The new position
            
        Returns:
            True if the move is valid, False otherwise
        """
        bridge_system = self.bridge_system
        
        # Check if the new position is within grid bounds
        if new_x < 0 or new_x >= bridge_system.width or new_y < 0 or new_y >= bridge_system.height:
            return False
        
        # Check if the new position is empty or a valid target
        new_pos_module = bridge_system.grid[new_y][new_x]
        if not new_pos_module:
            return False
            
        valid_dest_types = [ModuleType.STRUCTURE, 
                           ModuleType.BRIDGE,
                           ModuleType.TARGET]
                           
        if new_pos_module.type not in valid_dest_types:
            return False
        
        # Check if the new position is already occupied by a moving module
        if new_pos_module.moving_module is not None:
            return False
        
        # Make a temporary copy of the grid to check connectivity
        temp_grid = copy.deepcopy(bridge_system.grid)
        
        # Temporarily move the module
        old_x, old_y = module.x, module.y
        temp_grid[old_y][old_x] = None
        
        # Check if removing the module breaks connectivity for any non-source module
        # This is a simplified check - in practice, you'd need a more sophisticated
        # algorithm to verify the structure remains connected
        
        return True  # Simplified for now
    
    def execute_movement(self, module, path):
        """
        Execute a planned movement for a module along a path.
        This is a placeholder for the actual movement logic.
        
        Args:
            module: The module to move
            path: List of module IDs representing the path
            
        Returns:
            True if movement successful, False otherwise
        """
        # For now, just print the movement plan
        print(f"Moving module {module.id} along path: {' -> '.join(path)}")
        
        # In a real implementation, this would:
        # 1. Move the module step by step along the path
        # 2. Update the grid state after each step
        # 3. Handle collisions and waiting if needed
        
        return True 
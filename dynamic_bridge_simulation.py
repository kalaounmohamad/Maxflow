import os
import time
from datetime import datetime
import matplotlib.pyplot as plt

from dynamic_bridge_2d import DynamicBridgeSystem, ModuleType
from flow_analysis import FlowAnalyzer
from movement_rules import MovementPlanner

class BridgeSimulation:
    """Main simulation class that integrates all components"""
    
    def __init__(self, width=13, height=5):
        # Initialize the bridge system
        self.bridge_system = DynamicBridgeSystem(width, height)
        self.bridge_system.initialize_from_example()
        
        # Initialize the flow analyzer
        self.flow_analyzer = FlowAnalyzer(self.bridge_system)
        
        # Initialize the movement planner
        self.movement_planner = MovementPlanner(self.bridge_system, self.flow_analyzer)
        
        # Setup directory for saving images
        self.image_dir = os.path.join("images_2d", "simulation")
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
            
        # Current step in the simulation
        self.step = 0
        self.total_source_modules = len(self.bridge_system.source_modules)
        self.modules_at_target = 0
        
        # Keep track of bridge formations
        self.bridges_formed = []
    
    def run_initial_analysis(self):
        """Run initial flow analysis before any bridges are formed"""
        print("Running initial flow analysis...")
        
        # Calculate max flow and detect bottlenecks
        flow_value, _ = self.flow_analyzer.calculate_max_flow()
        bottlenecks = self.flow_analyzer.detect_bottlenecks()
        
        print(f"Initial max flow: {flow_value}")
        print(f"Detected {len(bottlenecks)} bottlenecks")
        
        # Visualize the initial state
        self.bridge_system.visualize_grid(
            title=f"Initial Configuration - Max Flow: {flow_value}",
            save_path=os.path.join(self.image_dir, "01_initial_grid.png")
        )
        
        # Visualize the flow network
        self.flow_analyzer.visualize_flow(
            title=f"Initial Flow Network - Max Flow: {flow_value}",
            save_path=os.path.join(self.image_dir, "02_initial_flow.png")
        )
        
        return flow_value, bottlenecks
    
    def plan_bridge_formation(self):
        """Plan the formation of bridges to improve flow"""
        print("\nPlanning bridge formations...")
        
        # Get bridge formation plans
        bridge_plans = self.movement_planner.plan_bridge_formations()
        
        if not bridge_plans:
            print("No bridge formations needed or possible.")
            return []
        
        print(f"Found {len(bridge_plans)} potential bridge formations")
        
        # For now, just return the plans
        # In a full simulation, we would execute these plans
        return bridge_plans
    
    def form_bridges(self, bridge_plans):
        """Form bridges according to the plans"""
        print("\nForming bridges...")
        
        if not bridge_plans:
            return 0
        
        # For now, let's just implement a simplified bridge formation
        # In reality, this would involve moving unused modules to form the bridge
        
        # Track how many bridges were formed
        bridges_formed = 0
        
        # Process each bridge plan
        for bridge_id, module_ids in bridge_plans:
            print(f"Forming bridge {bridge_id} with {len(module_ids)} modules")
            
            # Convert unused modules to bridge modules
            for module_id in module_ids:
                module = self.bridge_system.modules.get(module_id)
                if module and module.type == ModuleType.UNUSED:
                    # Convert to bridge
                    module.type = ModuleType.BRIDGE
                    bridges_formed += 1
                    
                    # Add to list of bridges formed
                    self.bridges_formed.append(module_id)
            
        print(f"Formed {bridges_formed} bridge modules")
        
        # Update the grid visualization after bridge formation
        self.bridge_system.visualize_grid(
            title=f"After Bridge Formation - {bridges_formed} Bridges",
            save_path=os.path.join(self.image_dir, f"03_bridges_formed_{self.step}.png")
        )
        
        return bridges_formed
    
    def recalculate_flow(self):
        """Recalculate flow after bridge formation"""
        print("\nRecalculating flow after bridge formation...")
        
        # Recalculate max flow
        flow_value, _ = self.flow_analyzer.calculate_max_flow()
        
        print(f"New max flow: {flow_value}")
        
        # Visualize the updated flow network
        self.flow_analyzer.visualize_flow(
            title=f"Updated Flow Network - Max Flow: {flow_value}",
            save_path=os.path.join(self.image_dir, f"04_updated_flow_{self.step}.png")
        )
        
        return flow_value, _
    
    def plan_source_movements(self):
        """Plan movements for source modules"""
        print("\nPlanning source module movements...")
        
        # Get source-target pairs with paths
        source_target_pairs = self.movement_planner.plan_source_movements()
        
        print(f"Planned movements for {len(source_target_pairs)} source modules")
        
        return source_target_pairs
    
    def execute_movements(self, source_target_pairs):
        """Execute the planned source module movements"""
        print("\nExecuting source module movements...")
        
        # In a real simulation, this would move modules step by step
        # For now, just simulate the final result
        
        moved_modules = 0
        
        for source_module, target_module, path in source_target_pairs:
            print(f"Moving {source_module.id} to {target_module.id} via path: {' -> '.join(path)}")
            
            # In a full simulation, we would:
            # 1. Move the module step by step
            # 2. Update the grid at each step
            # 3. Handle collisions and waiting
            
            # For now, just mark that this source has reached its target
            moved_modules += 1
        
        self.modules_at_target += moved_modules
        print(f"Moved {moved_modules} modules to targets")
        print(f"Total modules at targets: {self.modules_at_target}/{self.total_source_modules}")
        
        return moved_modules
    
    def run_simulation_step(self):
        """Run a single step of the simulation"""
        self.step += 1
        print(f"\n--- SIMULATION STEP {self.step} ---")
        
        # 1. Calculate current flow and detect bottlenecks
        flow_value, _ = self.flow_analyzer.calculate_max_flow()
        bottlenecks = self.flow_analyzer.detect_bottlenecks()
        
        # 2. If bottlenecks exist, plan bridge formations
        if bottlenecks:
            bridge_plans = self.plan_bridge_formation()
            bridges_formed = self.form_bridges(bridge_plans)
            
            # 3. If bridges were formed, recalculate flow
            if bridges_formed > 0:
                flow_value, _ = self.recalculate_flow()
        
        # 4. Plan and execute source movements
        source_target_pairs = self.plan_source_movements()
        moved_modules = self.execute_movements(source_target_pairs)
        
        # 5. Visualize the current state
        self.bridge_system.visualize_grid(
            title=f"Step {self.step} - {self.modules_at_target}/{self.total_source_modules} At Target",
            save_path=os.path.join(self.image_dir, f"05_step_{self.step}_grid.png")
        )
        
        return flow_value, moved_modules
    
    def run_full_simulation(self, max_steps=10):
        """Run the full simulation until all source modules reach targets or max steps"""
        print("\n=== STARTING FULL SIMULATION ===")
        
        # Run initial analysis
        initial_flow, initial_bottlenecks = self.run_initial_analysis()
        
        # Track metrics
        flow_values = [initial_flow]
        modules_at_target = [0]
        
        # Run simulation steps until all modules reach targets or max steps
        while self.modules_at_target < self.total_source_modules and self.step < max_steps:
            flow_value, moved_modules = self.run_simulation_step()
            
            # Track metrics
            flow_values.append(flow_value)
            modules_at_target.append(self.modules_at_target)
            
            # Break if no progress was made
            if moved_modules == 0 and len(self.bridges_formed) == 0:
                print("No progress made in this step. Ending simulation.")
                break
        
        # Display final results
        print("\n=== SIMULATION COMPLETE ===")
        print(f"Total steps: {self.step}")
        print(f"Modules at targets: {self.modules_at_target}/{self.total_source_modules}")
        print(f"Bridges formed: {len(self.bridges_formed)}")
        print(f"Final max flow: {flow_values[-1]}")
        
        # Plot metrics
        self.plot_metrics(flow_values, modules_at_target)
        
        return self.step, self.modules_at_target, flow_values[-1]
    
    def plot_metrics(self, flow_values, modules_at_target):
        """Plot metrics from the simulation"""
        plt.figure(figsize=(12, 8))
        
        # Plot max flow over time
        plt.subplot(2, 1, 1)
        plt.plot(range(len(flow_values)), flow_values, 'b-o', linewidth=2)
        plt.xlabel('Simulation Step')
        plt.ylabel('Max Flow')
        plt.title('Maximum Flow vs. Simulation Step')
        plt.grid(True)
        
        # Plot modules at target over time
        plt.subplot(2, 1, 2)
        plt.plot(range(len(modules_at_target)), modules_at_target, 'g-o', linewidth=2)
        plt.xlabel('Simulation Step')
        plt.ylabel('Modules at Target')
        plt.title('Modules at Target vs. Simulation Step')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.image_dir, "06_simulation_metrics.png"), dpi=300, bbox_inches='tight')
        plt.show()

# Example usage
if __name__ == "__main__":
    # Create and run the simulation
    simulation = BridgeSimulation()
    simulation.run_full_simulation(max_steps=5) 
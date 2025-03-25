import networkx as nx
import matplotlib.pyplot as plt
from networkx.algorithms.flow import edmonds_karp
from graph_configs import graph1, graph2, robot_graph, robot_graph_large
import os
from datetime import datetime

def create_graph(graph_data):
    G = nx.DiGraph()
    original_edges = [(u, v) for u, v, _ in graph_data]
    
    for u, v, capacity in graph_data:
        G.add_edge(u, v, capacity=capacity)
    
    return G, original_edges

def calculate_max_flow(G, source, sink, algorithm=edmonds_karp):
    flow_value, flow_dict = nx.maximum_flow(G, source, sink, flow_func=algorithm)
    
    # Update the graph with flow values
    for u in flow_dict:
        for v, flow in flow_dict[u].items():
            if G.has_edge(u, v):
                G[u][v]['flow'] = flow
    
    return flow_value, G

def print_flow_results(G, original_edges):
    print("\nFlow on each edge:")
    for u, v in original_edges:
        if 'flow' in G[u][v]:
            print(f"{u} -> {v}: {G[u][v]['flow']}/{G[u][v]['capacity']}")
        else:
            print(f"{u} -> {v}: 0/{G[u][v]['capacity']}")

def visualize_flow(G, original_edges, pos=None, title="Max Flow Network", filename="max_flow_network.png", 
                node_groups=None, subfolder=None):
    plt.figure(figsize=(12, 8))
    
    # Create a subgraph with only the original edges for visualization
    original_graph = nx.DiGraph()
    for u, v in original_edges:
        original_graph.add_edge(u, v)
    
    # If no positions are provided, use spring layout
    if pos is None:
        pos = nx.spring_layout(original_graph)
    
    # Default node coloring if no groups specified
    if node_groups is None:
        nx.draw(original_graph, pos, with_labels=True, node_size=3000, node_color='lightblue', 
                font_size=15, font_weight='bold', arrows=True)
    else:
        # Draw nodes by group with different colors
        for group_name, (nodes, color) in node_groups.items():
            nx.draw_networkx_nodes(original_graph, pos, 
                                  nodelist=[n for n in nodes if n in original_graph.nodes()], 
                                  node_color=color, node_size=3000, label=group_name)
        
        # Draw edges
        nx.draw_networkx_edges(original_graph, pos, arrows=True, width=1.5)
        
        # Draw labels
        nx.draw_networkx_labels(original_graph, pos, font_size=15, font_weight='bold')
        
        # Add legend
        plt.legend(loc='upper right')
    
    # Draw edge labels (capacity and flow) only for original edges
    edge_labels = {}
    for u, v in original_edges:
        if 'flow' in G[u][v]:
            edge_labels[(u, v)] = f"{G[u][v]['flow']}/{G[u][v]['capacity']}"
        else:
            edge_labels[(u, v)] = f"0/{G[u][v]['capacity']}"
    
    nx.draw_networkx_edge_labels(original_graph, pos, edge_labels=edge_labels, font_size=12)
    
    plt.title(title)
    plt.axis('off')  # Turn off the axis
    plt.tight_layout()
    
    # Save if filename provided
    if filename:
        # Create timestamp for unique filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Determine save path
        if subfolder:
            # Create the directory if it doesn't exist
            save_dir = os.path.join("images_2d", subfolder)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # Add timestamp to filename
            base_name = os.path.splitext(filename)[0]
            ext = os.path.splitext(filename)[1] or ".png"
            save_path = os.path.join(save_dir, f"{base_name}_{timestamp}{ext}")
        else:
            # Save in main images_2d folder if no subfolder specified
            if not os.path.exists("images_2d"):
                os.makedirs("images_2d")
            
            # Add timestamp to filename
            base_name = os.path.splitext(filename)[0]
            ext = os.path.splitext(filename)[1] or ".png"
            save_path = os.path.join("images_2d", f"{base_name}_{timestamp}{ext}")
        
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved 2D visualization to: {save_path}")
        
    plt.show()

def analyze_flow_graph(graph):
    """Analyze a FlowGraph object"""
    print(f"\n{graph.name} ANALYSIS")
    G, original_edges = create_graph(graph.edges)
    flow_value, G = calculate_max_flow(G, graph.source, graph.sink)
    
    print(f"Maximum flow: {flow_value}")
    print_flow_results(G, original_edges)
    
    # Extract graph type for subfolder
    graph_type = graph.name.lower().replace(' ', '_').split('_')[0]
    visualize_flow(G, original_edges, graph.positions, graph.title, 
                  f"flow_{graph_type}.png", graph.node_groups, subfolder=graph_type)
    
    return flow_value, G

if __name__ == "__main__":    
    # Analyze graphs
    # analyze_flow_graph(graph1)
    # analyze_flow_graph(graph2)
    
    # Original robot graph with unit capacities
    print("\n--- ROBOT GRAPH WITH UNIT CAPACITIES ---")
    analyze_flow_graph(robot_graph)
    
    # Robot graph with large capacities
    # print("\n--- ROBOT GRAPH WITH LARGE CAPACITIES ---")
    # analyze_flow_graph(robot_graph_large)
    
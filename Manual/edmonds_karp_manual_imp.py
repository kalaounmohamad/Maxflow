import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
from graph_configs import ALL_CONFIGS

def edmonds_karp(G, source, sink):
    """
    Implements the Edmonds-Karp algorithm to find the maximum flow in a network.
    
    Args:
        G: NetworkX DiGraph with capacity attributes on edges
        source: Source node
        sink: Sink node
        
    Returns:
        Maximum flow value
    """
    # Initialize flow to 0 and ensure reverse edges exist
    for u, v in list(G.edges()):
        G[u][v]['flow'] = 0
        # Add reverse edge if it doesn't exist
        if not G.has_edge(v, u):
            G.add_edge(v, u, capacity=0, flow=0)
    
    max_flow = 0
    
    # While there exists an augmenting path
    while True:
        # Find an augmenting path using BFS
        path, min_capacity = bfs(G, source, sink)
        
        if not path:
            break  # No augmenting path found, we're done
            
        # Update flow along the path
        max_flow += min_capacity
        v = sink
        while v != source:
            u = path[v]
            G[u][v]['flow'] += min_capacity
            G[v][u]['flow'] -= min_capacity
            v = u
    
    return max_flow

def bfs(G, source, sink):
    """
    Use BFS to find an augmenting path from source to sink.
    
    Returns:
        (path, min_capacity) where path is a dict of {node: predecessor}
        and min_capacity is the minimum residual capacity along the path
    """
    queue = deque([source])
    path = {source: None}
    
    # Track minimum capacity along the path
    capacity = {source: float('inf')}
    
    while queue and sink not in path: ## Main loop stops if sink found
        u = queue.popleft()
        
        for v in G.neighbors(u):
            # Check if there's residual capacity and node not visited
            residual = G[u][v]['capacity'] - G[u][v]['flow']
            if residual > 0 and v not in path:
                path[v] = u
                # Update capacity while exploring the graph
                capacity[v] = min(capacity[u], residual)
                queue.append(v)
                
                if v == sink:
                    break ## Exit the neighbor exploration loop when sink is found
    
    if sink in path:
        return path, capacity[sink]
    else:
        return None, 0

# Choose configuration (change this to use different configs)
config_choice = 11
config = ALL_CONFIGS[config_choice]

print(f"Using configuration: {config['name']}")

# Create the graph from configuration
G = nx.DiGraph()

# Get edges, source, sink, and positions from config
edges = config['edges']
source = config['source']
sink = config['sink']
pos = config['positions']

# Store original edges to use for visualization later
original_edges = [(u, v) for u, v, _ in edges]

for u, v, capacity in edges:
    G.add_edge(u, v, capacity=capacity, flow=0)

# Calculate max flow
max_flow = edmonds_karp(G, source, sink)
print(f"Maximum flow: {max_flow}")

# Print the flow on each edge (only original edges)
print("\nFlow on each edge:")
for u, v in original_edges:
    print(f"{u} -> {v}: {G[u][v]['flow']}/{G[u][v]['capacity']}")

# Draw the graph
plt.figure(figsize=(12, 8))

# Create a subgraph with only the original edges for visualization
original_graph = nx.DiGraph()
for u, v in original_edges:
    original_graph.add_edge(u, v)

# Use custom node colors
node_colors = []
for node in original_graph.nodes():
    if node == 'Super_S':
        node_colors.append('yellow')
    elif node == 'Super_T':
        node_colors.append('yellow')
    elif node.startswith('S_'):
        node_colors.append('red')
    elif node.startswith('T_'):
        node_colors.append('green')
    elif node.startswith('B_'):
        node_colors.append('orange')
    else:
        node_colors.append('lightblue')

nx.draw(original_graph, pos, with_labels=True, node_size=1500, node_color=node_colors,
        font_size=10, font_weight='bold', arrows=True)

# Draw edge labels (capacity and flow) only for original edges
edge_labels = {(u, v): f"{G[u][v]['flow']}/{G[u][v]['capacity']}" 
               for u, v in original_edges}
nx.draw_networkx_edge_labels(original_graph, pos, edge_labels=edge_labels, font_size=8)

plt.title(f"Max Flow Network - {config_choice +1} - {config['name']}")
plt.savefig(f"max_flow_{config_choice +1}_{config['name'].lower().replace(' ', '_')}.png", dpi=300)
plt.show()

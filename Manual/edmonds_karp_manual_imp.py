import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
from graph_configs import ALL_CONFIGS
import heapq          # ← priority-queue for Dijkstra

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

def bfs_pq_lexico_sum(G, source, sink):
    """
    Return (predecessor_dict, bottleneck_capacity) for the residual graph,
    using lexicographic cost:
        1) minimise hop count
        2) if equal hops, minimise Σ(existing flow) along the path.
    """
    # cost[node]  = (hops_so_far, sum_flow_so_far)
    cost = {source: (0, 0)}
    pred = {source: None}
    pq = [(0, 0, source)]       # (hops, sum_flow, node)

    while pq:
        hops_so_far, flow_so_far, u = heapq.heappop(pq)

        if u == sink:
            # reconstruct min residual capacity along the path
            v = sink
            bottleneck = float('inf')
            while pred[v] is not None:
                p = pred[v]
                residual = G[p][v]['capacity'] - G[p][v]['flow']
                bottleneck = min(bottleneck, residual)
                v = p
            return pred, bottleneck

        # skip if we already found a better way to u
        if (hops_so_far, flow_so_far) != cost[u]:
            continue

        for v in G.neighbors(u):
            residual = G[u][v]['capacity'] - G[u][v]['flow']
            if residual <= 0:
                continue                      # saturated edge, ignore

            new_cost = (hops_so_far + 1,     # +1 hop
                        flow_so_far + G[u][v]['flow'])  # add existing flow

            if v not in cost or new_cost < cost[v]:
                cost[v] = new_cost
                pred[v] = u
                heapq.heappush(pq, (*new_cost, v))

    # no path
    return None, 0

# Choose configuration (change this to use different configs)
config_choice = 13
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

# Create visualization of original network before flow calculation
plt.figure(figsize=(12, 8))
original_graph_before_flow = nx.DiGraph()

# Add all nodes and edges from original configuration
for u, v, capacity in edges:
    original_graph_before_flow.add_node(u)
    original_graph_before_flow.add_node(v)
    original_graph_before_flow.add_edge(u, v)

# Use custom node colors (same as in the final visualization)
node_colors_original = []
for node in original_graph_before_flow.nodes():
    if node == 'Super_S':
        node_colors_original.append('yellow')
    elif node == 'Super_T':
        node_colors_original.append('yellow')
    elif node.startswith('S_'):
        node_colors_original.append('red')
    elif node.startswith('T_'):
        node_colors_original.append('green')
    elif node.startswith('B_'):
        node_colors_original.append('orange')
    else:
        node_colors_original.append('lightblue')

# Draw nodes
nx.draw_networkx_nodes(original_graph_before_flow, pos, node_color=node_colors_original, 
                      node_size=1500, label=None)
nx.draw_networkx_labels(original_graph_before_flow, pos, font_size=10, font_weight='bold')

# Draw edges with a single color
nx.draw_networkx_edges(original_graph_before_flow, pos, arrows=True)

# Draw edge labels showing only capacities
edge_labels_orig = {(u, v): f"{G[u][v]['capacity']}" for u, v in original_edges if G[u][v]['capacity'] > 0}
nx.draw_networkx_edge_labels(original_graph_before_flow, pos, edge_labels=edge_labels_orig, font_size=8)

plt.title(f"Original Network - {config_choice +1} - {config['name']}")
plt.savefig(f"original_{config_choice +1}_{config['name'].lower().replace(' ', '_')}.png", dpi=300)
plt.close()  # Close the figure to avoid displaying it twice

# Calculate max flow
max_flow = edmonds_karp(G, source, sink)
print(f"Maximum flow: {max_flow}")

# Print the flow on each edge (only original edges)
print("\nFlow on each edge:")
for u, v in original_edges:
    print(f"{u} -> {v}: {G[u][v]['flow']}/{G[u][v]['capacity']}")

# Draw the graph
plt.figure(figsize=(12, 8))

# Create a subgraph that includes all nodes but only non-zero flow edges
original_graph = nx.DiGraph()

# First, add all nodes to ensure they appear even without active flows
for u, v, _ in edges:
    original_graph.add_node(u)
    original_graph.add_node(v)

# Init edge visualization lists
edges_to_draw = []
edge_colors = []
edge_widths = []
max_flow_ratio = 0.01  # Initialize to small number to avoid division by zero

# First pass to find max flow ratio for normalization - exclude super source/sink edges
for u, v in original_edges:
    # Skip super source/sink edges for color normalization
    if (u.startswith('Super_') or v.startswith('Super_') or 
        G[u][v]['capacity'] <= 0 or G[u][v]['flow'] <= 0):
        continue
        
    flow_ratio = G[u][v]['flow'] / G[u][v]['capacity']
    max_flow_ratio = max(max_flow_ratio, flow_ratio)

# Add edges and calculate colors
for u, v in original_edges:
    if G[u][v]['capacity'] > 0 and G[u][v]['flow'] > 0:  # Skip zero capacity or zero flow edges
        original_graph.add_edge(u, v)
        edges_to_draw.append((u, v))
        
        # Calculate color intensity based on flow/capacity ratio
        flow_ratio = G[u][v]['flow'] / G[u][v]['capacity']
        
        # Use a standard color for super source/sink edges
        if u.startswith('Super_') or v.startswith('Super_'):
            edge_colors.append((0.7, 0.7, 0.7, 0.8))  # Gray color
            edge_widths.append(1.5)  # Standard width
        else:
            # Normal edges: Normalize by max ratio for better color contrast
            normalized_ratio = flow_ratio / max_flow_ratio
            
            # Edge with minimum flow is light blue, full flow is dark blue
            edge_colors.append((0.1, 0.1, 0.8, 0.3 + 0.7 * normalized_ratio))
            
            # Make edge width proportional to flow amount
            edge_widths.append(1 + 2 * normalized_ratio)

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

# Draw nodes
nx.draw_networkx_nodes(original_graph, pos, node_color=node_colors, 
                       node_size=1500, label=None)
nx.draw_networkx_labels(original_graph, pos, font_size=10, font_weight='bold')

# Draw edges with color coding
nx.draw_networkx_edges(original_graph, pos, edgelist=edges_to_draw, 
                       width=edge_widths, edge_color=edge_colors, arrows=True)

# Draw edge labels (capacity and flow) only for non-zero flow original edges
edge_labels = {(u, v): f"{G[u][v]['flow']}/{G[u][v]['capacity']}" 
               for u, v in original_edges if G[u][v]['capacity'] > 0 and G[u][v]['flow'] > 0}
nx.draw_networkx_edge_labels(original_graph, pos, edge_labels=edge_labels, font_size=8)

plt.title(f"Max Flow Network - {config_choice +1} - {config['name']}")
plt.savefig(f"max_flow_{config_choice +1}_{config['name'].lower().replace(' ', '_')}.png", dpi=300)
plt.show()

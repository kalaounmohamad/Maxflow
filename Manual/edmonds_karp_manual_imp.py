import networkx as nx
import matplotlib.pyplot as plt
from collections import deque

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
    
    while queue and sink not in path:
        u = queue.popleft()
        
        for v in G.neighbors(u):
            # Check if there's residual capacity and node not visited
            residual = G[u][v]['capacity'] - G[u][v]['flow']
            if residual > 0 and v not in path:
                path[v] = u
                capacity[v] = min(capacity[u], residual)
                queue.append(v)
                
                if v == sink:
                    break
    
    if sink in path:
        return path, capacity[sink]
    else:
        return None, 0

# Create the graph as shown in the image
G = nx.DiGraph()

# Add edges with capacities (source, target, capacity)
edges = [
    ('S', 'S1', float('inf')),
    ('S', 'S2', float('inf')),
    ('S1', 'B', 7),
    ('S1', 'A', 5),
    ('S2', 'A', 7),
    ('B', 'D', 10),
    ('B', 'C', 5),
    ('A', 'C', 19),
    ('C', 'E', 27),
    ('D', 'T', 12),
    ('E', 'T', 15)
]

# Store original edges to use for visualization later
original_edges = [(u, v) for u, v, _ in edges]

for u, v, capacity in edges:
    G.add_edge(u, v, capacity=capacity, flow=0)

# Calculate max flow
max_flow = edmonds_karp(G, 'S', 'T')
print(f"Maximum flow: {max_flow}")

# Print the flow on each edge (only original edges)
print("\nFlow on each edge:")
for u, v in original_edges:
    print(f"{u} -> {v}: {G[u][v]['flow']}/{G[u][v]['capacity']}")

# Draw the graph
pos = {
    'S': (0, 1),
    'S1': (1, 2),
    'S2': (1, 0),
    'A': (1, 1),
    'B': (2, 2),
    'C': (2, 1),
    'D': (3, 2),
    'E': (3, 1),
    'T': (4, 2)
}

plt.figure(figsize=(10, 6))

# Create a subgraph with only the original edges for visualization
original_graph = nx.DiGraph()
for u, v in original_edges:
    original_graph.add_edge(u, v)

nx.draw(original_graph, pos, with_labels=True, node_size=3000, node_color='lightblue', 
        font_size=15, font_weight='bold', arrows=True)

# Draw edge labels (capacity and flow) only for original edges
edge_labels = {(u, v): f"{G[u][v]['flow']}/{G[u][v]['capacity']}" 
               for u, v in original_edges}
nx.draw_networkx_edge_labels(original_graph, pos, edge_labels=edge_labels, font_size=12)

plt.title("Max Flow Network")
plt.savefig("max_flow_network.png")
plt.show()

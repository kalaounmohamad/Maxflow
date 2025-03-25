import networkx as nx
import matplotlib.pyplot as plt
from networkx.algorithms.flow import edmonds_karp

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
    G.add_edge(u, v, capacity=capacity)

# Calculate max flow using NetworkX's implementation
flow_value, flow_dict = nx.maximum_flow(G, 'S', 'T', flow_func=edmonds_karp)
print(f"Maximum flow: {flow_value}")

# Update the graph with flow values
for u in flow_dict:
    for v, flow in flow_dict[u].items():
        if G.has_edge(u, v):
            G[u][v]['flow'] = flow

# Print the flow on each edge (only original edges)
print("\nFlow on each edge:")
for u, v in original_edges:
    if 'flow' in G[u][v]:
        print(f"{u} -> {v}: {G[u][v]['flow']}/{G[u][v]['capacity']}")
    else:
        print(f"{u} -> {v}: 0/{G[u][v]['capacity']}")

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
edge_labels = {}
for u, v in original_edges:
    if 'flow' in G[u][v]:
        edge_labels[(u, v)] = f"{G[u][v]['flow']}/{G[u][v]['capacity']}"
    else:
        edge_labels[(u, v)] = f"0/{G[u][v]['capacity']}"

nx.draw_networkx_edge_labels(original_graph, pos, edge_labels=edge_labels, font_size=12)

plt.title("Max Flow Network")
plt.savefig("max_flow_network.png")
plt.show()

import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
import heapq  # For priority queue
from graph_configs import ALL_CONFIGS

def edmonds_karp(G, source, sink):
    """
    Implements a modified Edmonds-Karp algorithm that prioritizes paths with
    least residual flow and shorter distances.
    
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
    path_count = 0
    
    # While there exists an augmenting path
    while True:
        # Find an augmenting path using priority-based path finding
        path, min_capacity = priority_path_finding(G, source, sink)
        
        if not path:
            break  # No augmenting path found, we're done
            
        # Update flow along the path
        max_flow += min_capacity
        path_count += 1
        print(f"Path {path_count}: Augmented flow by {min_capacity}")
        
        v = sink
        while v != source:
            u = path[v]
            G[u][v]['flow'] += min_capacity
            G[v][u]['flow'] -= min_capacity
            v = u
    
    return max_flow

def priority_path_finding(G, source, sink):
    """
    Find an augmenting path using virtual laser guidance:
    1. Each source identifies its closest target based on Euclidean distance
    2. Creates a virtual laser line from source to target
    3. Prioritizes edges that align best with this virtual path
    4. Balances flow as secondary priority
    
    Returns:
        (path, min_capacity) where path is a dict of {node: predecessor}
        and min_capacity is the minimum residual capacity along the path
    """
    # Priority queue: (priority, node, capacity_so_far, virtual_target)
    # Lower priority value = higher priority
    
    queue = [(0, source, float('inf'), None)]
    path = {source: None}
    capacity = {source: float('inf')}
    visited = set()
    
    while queue and sink not in visited:
        # Get the highest priority element
        _, current, cap_so_far, virtual_target = heapq.heappop(queue)
        
        if current in visited:
            continue
            
        visited.add(current)
        
        # Find virtual target based on current node's position
        if virtual_target is None or current.startswith('S_'):
            # For source nodes, find their corresponding target
            virtual_target = find_corresponding_target(current, G, pos)
        
        for neighbor in G.neighbors(current):
            residual = G[current][neighbor]['capacity'] - G[current][neighbor]['flow']
            
            if residual > 0 and neighbor not in visited:
                new_capacity = min(cap_so_far, residual)
                
                # Get current flow on this edge for balancing
                current_flow = G[current][neighbor]['flow']
                
                # Calculate how well this edge aligns with the virtual laser path
                if virtual_target:
                    alignment_score = calculate_laser_alignment(current, neighbor, virtual_target, pos)
                else:
                    alignment_score = 0  # Default for super source/sink paths
                
                # Priority formula - lower is better
                # 1. Laser alignment is primary
                # 2. Flow balancing is secondary
                # 3. Residual capacity is tie-breaker
                laser_weight = 2
                flow_weight = 1
                priority = (alignment_score * laser_weight) + (current_flow * flow_weight) + (1.0 / (residual + 0.1))
                
                heapq.heappush(queue, (priority, neighbor, new_capacity, virtual_target))
                
                if neighbor not in path:
                    path[neighbor] = current
                    capacity[neighbor] = new_capacity
    
    if sink in path:
        return path, capacity[sink]
    else:
        return None, 0

def find_corresponding_target(source_node, G, pos):
    """
    Find the target that corresponds to the same row as the source.
    For S_0 -> T_0, S_1 -> T_1, etc.
    """
    if not source_node.startswith('S_'):
        return find_closest_target(source_node, G, pos)
    
    # Extract the row number from source node (e.g., 'S_2' -> '2')
    try:
        row_num = source_node.split('_')[1]
        target_node = f'T_{row_num}'
        
        # Verify this target exists in the graph
        if target_node in G.nodes() and target_node in pos:
            return target_node
        else:
            # Fallback to closest target
            return find_closest_target(source_node, G, pos)
    except:
        return find_closest_target(source_node, G, pos)

def find_closest_target(source_node, G, pos):
    """
    Find the closest target node to the given source based on Euclidean distance.
    
    Args:
        source_node: Current source node
        G: Graph
        pos: Node positions dictionary
        
    Returns:
        Closest target node name
    """
    if source_node not in pos:
        return None
    
    source_pos = pos[source_node]
    min_distance = float('inf')
    closest_target = None
    
    # Look for target nodes (nodes starting with 'T_')
    for node in G.nodes():
        if node.startswith('T_') and node in pos:
            target_pos = pos[node]
            distance = ((source_pos[0] - target_pos[0])**2 + (source_pos[1] - target_pos[1])**2)**0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_target = node
    
    return closest_target

def calculate_laser_alignment(current_node, next_node, target_node, pos):
    """
    Calculate how well the edge (current->next) aligns with the virtual laser path (current->target).
    Uses pure vector alignment without penalizing specific movement directions.
    """
    if not all(node in pos for node in [current_node, next_node, target_node]):
        return 1000  # High penalty for missing position data
    
    current_pos = pos[current_node]
    next_pos = pos[next_node]
    target_pos = pos[target_node]
    
    # Vector from current to next (actual edge direction)
    edge_vector = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1])
    
    # Vector from current to target (desired laser direction)
    laser_vector = (target_pos[0] - current_pos[0], target_pos[1] - current_pos[1])
    
    # Normalize vectors to unit length
    edge_length = (edge_vector[0]**2 + edge_vector[1]**2)**0.5
    laser_length = (laser_vector[0]**2 + laser_vector[1]**2)**0.5
    
    if edge_length == 0 or laser_length == 0:
        return 1000  # High penalty for zero-length vectors
    
    edge_unit = (edge_vector[0]/edge_length, edge_vector[1]/edge_length)
    laser_unit = (laser_vector[0]/laser_length, laser_vector[1]/laser_length)
    
    # Calculate dot product (cosine of angle between vectors)
    dot_product = edge_unit[0]*laser_unit[0] + edge_unit[1]*laser_unit[1]
    
    # Convert to alignment score: 1 - dot_product
    # dot_product = 1 (perfect alignment) -> score = 0 (best priority)
    # dot_product = -1 (opposite direction) -> score = 2 (worst priority)
    alignment_score = 1 - dot_product
    
    # Bonus for making progress towards target
    current_to_target_dist = laser_length
    next_to_target_dist = ((next_pos[0] - target_pos[0])**2 + (next_pos[1] - target_pos[1])**2)**0.5
    
    # Bonus for getting closer to target in any direction
    progress_bonus = max(0, current_to_target_dist - next_to_target_dist) / (current_to_target_dist + 0.1)
    
    return alignment_score - (progress_bonus * 2)  # Double the progress bonus

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
print("Running modified Edmonds-Karp with priority-based path finding...")
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

# Add positions for all nodes
nx.set_node_attributes(original_graph, {node: pos[node] for node in original_graph.nodes()}, 'pos')

# Init edge visualization lists
edges_to_draw = []
edge_colors = []
edge_widths = []
max_flow_ratio = 0.01  # Initialize to small number to avoid division by zero

# First pass to find max flow ratio for normalization - exclude super source/sink edges
for u, v in original_edges:
    # Skip super source/sink edges for color normalization
    if (u == 'Super_S' or v == 'Super_T' or 
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
        if u == 'Super_S' or v == 'Super_T':
            edge_colors.append((0.7, 0.7, 0.7, 0.8))  # Gray color
            edge_widths.append(1.5)  # Standard width
        else:
            # Normal edges: Normalize by max ratio for better color contrast
            normalized_ratio = flow_ratio / max_flow_ratio
            
            # Edge with minimum flow is light blue, full flow is dark blue
            edge_colors.append((0.1, 0.1, 0.8, 0.3 + 0.7 * normalized_ratio))
            
            # Make edge width proportional to flow amount
            edge_widths.append(1 + 2 * normalized_ratio)

# Use custom node colors (unchanged)
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

plt.title(f"Modified Max Flow - {config_choice+1} - {config['name']}")
plt.savefig(f"modified_max_flow_{config_choice+1}_{config['name'].lower().replace(' ', '_')}.png", dpi=300)
plt.show()

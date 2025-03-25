class FlowGraph:
    def __init__(self, name, edges, positions, node_groups, source, sink):
        self.name = name
        self.edges = edges
        self.positions = positions
        self.node_groups = node_groups
        self.source = source
        self.sink = sink
        self.title = f"{name} - Max Flow"
        self.filename = f"{name.lower().replace(' ', '_')}_flow.png"

# Example graph
graph1 = FlowGraph(
    name="Graph 1",
    edges=[
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
    ],
    positions={
        'S': (0, 1),
        'S1': (1, 2),
        'S2': (1, 0),
        'A': (1, 1),
        'B': (2, 2),
        'C': (2, 1),
        'D': (3, 2),
        'E': (3, 1),
        'T': (4, 2)
    },
    node_groups={
        'Source': (['S'], 'gold'),
        'Intermediate 1': (['S1', 'S2'], 'lightgreen'),
        'Intermediate 2': (['A', 'B'], 'skyblue'),
        'Intermediate 3': (['C', 'D', 'E'], 'lightcoral'),
        'Sink': (['T'], 'purple')
    },
    source='S',
    sink='T'
)

# Diamond network
graph2 = FlowGraph(
    name="Diamond Network",
    edges=[
        ('S', 'A', 10),
        ('S', 'B', 8),
        ('A', 'C', 5),
        ('A', 'D', 7),
        ('B', 'C', 6),
        ('B', 'D', 9),
        ('C', 'T', 12),
        ('D', 'T', 15)
    ],
    positions={
        'S': (0, 1),
        'A': (1, 1.5),
        'B': (1, 0.5),
        'C': (2, 2),
        'D': (2, 0.5),
        'T': (3, 1)
    },
    node_groups={
        'Source': (['S'], 'gold'),
        'Layer 1': (['A', 'B'], 'skyblue'),
        'Layer 2': (['C', 'D'], 'lightcoral'),
        'Sink': (['T'], 'purple')
    },
    source='S',
    sink='T'
)

# Multi-source, multi-target robot scenario
robot_graph = FlowGraph(
    name="Modular Robots",
    edges=[
        # Connect super source to real sources with infinite capacity
        ('Super_S', 'S1', 1),
        ('Super_S', 'S2', 1),
        ('Super_S', 'S3', 1),
        
        # First layer of connections
        ('S1', 'A1', 1),
        ('S2', 'A1', 1),
        ('S3', 'A1', 1),

        # Second layer of connections
        ('A1', 'B1', 1),
        ('A1', 'B2', 1),
        ('A1', 'B3', 1),
        ('A1', 'B4', 1),
        ('A1', 'B5', 1),
        
        # Connect to targets
        ('B1', 'T1', 1),
        ('B2', 'T1', 1),
        ('B2', 'T2', 1),
        ('B3', 'T2', 1),
        ('B3', 'T3', 1),
        ('B4', 'T3', 1),
        ('B5', 'T3', 1),
        
        # Connect targets to super sink with infinite capacity
        ('T1', 'Super_T', 1),
        ('T2', 'Super_T', 1),
        ('T3', 'Super_T', 1)
    ],
    positions={
        'Super_S': (0, 3),
        'S1': (1, 5),
        'S2': (1, 3),
        'S3': (1, 1),
        'A1': (2, 5),
        'B1': (3, 5.5),
        'B2': (3, 4.5),
        'B3': (3, 3),
        'B4': (3, 1.5),
        'B5': (3, 0.5),
        'T1': (4, 5),
        'T2': (4, 3),
        'T3': (4, 1),
        'Super_T': (5, 3)
    },
    node_groups={
        'Super Nodes': (['Super_S', 'Super_T'], 'gold'),
        'Source Nodes': (['S1', 'S2', 'S3'], 'lightgreen'),
        'Layer A': (['A1', 'A2', 'A3', 'A4'], 'skyblue'),
        'Layer B': (['B1', 'B2', 'B3', 'B4', 'B5'], 'lightcoral'),
        'Target Nodes': (['T1', 'T2', 'T3'], 'purple')
    },
    source='Super_S',
    sink='Super_T'
)

# Multi-source, multi-target robot scenario with large but finite capacity edges
robot_graph_large = FlowGraph(
    name="Modular Robots (Large Capacity)",
    edges=[
        # Connect super source to real sources with infinite capacity
        ('Super_S', 'S1', 1),
        ('Super_S', 'S2', 1),
        ('Super_S', 'S3', 1),
        
        # First layer of connections (large but finite capacity)
        ('S1', 'A1', 1000),
        ('S1', 'A2', 1000), 
        ('S2', 'A2', 1000),
        ('S2', 'A3', 1000),
        ('S3', 'A3', 1000),
        ('S3', 'A4', 1000),
        
        # Second layer of connections (large but finite capacity)
        ('A1', 'B1', 1000),
        ('A1', 'B2', 1000),
        ('A2', 'B2', 1000),
        ('A2', 'B3', 1000),
        ('A3', 'B3', 1000),
        ('A3', 'B4', 1000),
        ('A4', 'B4', 1000),
        ('A4', 'B5', 1000),
        
        # Connect to targets (large but finite capacity)
        ('B1', 'T1', 1000),
        ('B2', 'T1', 1000),
        ('B2', 'T2', 1000),
        ('B3', 'T2', 1000),
        ('B3', 'T3', 1000),
        ('B4', 'T3', 1000),
        ('B5', 'T3', 1000),
        
        # Connect targets to super sink with infinite capacity
        ('T1', 'Super_T', 1),
        ('T2', 'Super_T', 1),
        ('T3', 'Super_T', 1)
    ],
    positions={
        'Super_S': (0, 3),
        'S1': (1, 5),
        'S2': (1, 3),
        'S3': (1, 1),
        'A1': (2, 5),
        'A2': (2, 4),
        'A3': (2, 2),
        'A4': (2, 1),
        'B1': (3, 5.5),
        'B2': (3, 4.5),
        'B3': (3, 3),
        'B4': (3, 1.5),
        'B5': (3, 0.5),
        'T1': (4, 5),
        'T2': (4, 3),
        'T3': (4, 1),
        'Super_T': (5, 3)
    },
    node_groups={
        'Super Nodes': (['Super_S', 'Super_T'], 'gold'),
        'Source Nodes': (['S1', 'S2', 'S3'], 'lightgreen'),
        'Layer A': (['A1', 'A2', 'A3', 'A4'], 'skyblue'),
        'Layer B': (['B1', 'B2', 'B3', 'B4', 'B5'], 'lightcoral'),
        'Target Nodes': (['T1', 'T2', 'T3'], 'purple')
    },
    source='Super_S',
    sink='Super_T'
) 
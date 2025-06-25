# Network configurations for Edmonds-Karp algorithm testing

# Configuration 1: Basic network from the main example
CONFIG_1 = {
    'name': 'Basic Network',
    'edges': [
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
    'source': 'S',
    'sink': 'T',
    'positions': {
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
}

# Configuration 2: Simple linear network
CONFIG_2 = {
    'name': 'Linear Network',
    'edges': [
        ('A', 'B', 10),
        ('B', 'C', 8),
        ('C', 'D', 6),
        ('D', 'E', 4)
    ],
    'source': 'A',
    'sink': 'E',
    'positions': {
        'A': (0, 0),
        'B': (1, 0),
        'C': (2, 0),
        'D': (3, 0),
        'E': (4, 0)
    }
}

# Configuration 3: Diamond network
CONFIG_3 = {
    'name': 'Diamond Network',
    'edges': [
        ('S', 'A', 16),
        ('S', 'B', 13),
        ('A', 'B', 4),
        ('A', 'C', 12),
        ('B', 'D', 14),
        ('C', 'T', 20),
        ('D', 'T', 4),
        ('C', 'D', 9)
    ],
    'source': 'S',
    'sink': 'T',
    'positions': {
        'S': (0, 1),
        'A': (1, 2),
        'B': (1, 0),
        'C': (2, 2),
        'D': (2, 0),
        'T': (3, 1)
    }
}

# Configuration 4: Complex network
CONFIG_4 = {
    'name': 'Complex Network',
    'edges': [
        ('S', 'A', 10),
        ('S', 'B', 10),
        ('A', 'B', 2),
        ('A', 'C', 4),
        ('A', 'D', 8),
        ('B', 'D', 9),
        ('C', 'T', 10),
        ('D', 'C', 6),
        ('D', 'T', 10)
    ],
    'source': 'S',
    'sink': 'T',
    'positions': {
        'S': (0, 1),
        'A': (1, 2),
        'B': (1, 0),
        'C': (2, 2),
        'D': (2, 0),
        'T': (3, 1)
    }
}

# Configuration 5: Grid Network (based on dynamic bridge system)
CONFIG_5 = {
    'name': 'Grid Network',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', float('inf')),
        ('Super_S', 'S_1', float('inf')),
        ('Super_S', 'S_2', float('inf')),
        ('Super_S', 'S_3', float('inf')),
        ('Super_S', 'S_4', float('inf')),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', float('inf')),
        ('T_1', 'Super_T', float('inf')),
        ('T_2', 'Super_T', float('inf')),
        ('T_3', 'Super_T', float('inf')),
        ('T_4', 'Super_T', float('inf')),
        
        # Connect source nodes to left structure
        ('S_0', 'M_L_0', float('inf')),
        ('S_1', 'M_L_1', float('inf')),
        ('S_2', 'M_L_2', float('inf')),
        ('S_3', 'M_L_3', float('inf')),
        ('S_4', 'M_L_4', float('inf')),
        
        # Connect left structure nodes vertically
        ('M_L_0', 'M_L_1', float('inf')),
        ('M_L_1', 'M_L_2', float('inf')),
        ('M_L_2', 'M_L_3', float('inf')),
        ('M_L_3', 'M_L_4', float('inf')),
        
        # Connect right structure nodes vertically
        ('M_R_0', 'M_R_1', float('inf')),
        ('M_R_1', 'M_R_2', float('inf')),
        ('M_R_2', 'M_R_3', float('inf')),
        ('M_R_3', 'M_R_4', float('inf')),
        
        # Connect bridge nodes between structures
        ('M_L_2', 'B_0', float('inf')),
        ('B_0', 'B_1', float('inf')),
        ('B_1', 'M_R_2', float('inf')),
        
        # Connect right structure to targets
        ('M_R_0', 'T_0', float('inf')),
        ('M_R_1', 'T_1', float('inf')),
        ('M_R_2', 'T_2', float('inf')),
        ('M_R_3', 'T_3', float('inf')),
        ('M_R_4', 'T_4', float('inf')),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (6, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Bridge nodes
        'B_0': (2, 2),
        'B_1': (3, 2),
        
        # Right structure nodes
        'M_R_0': (4, 0),
        'M_R_1': (4, 1),
        'M_R_2': (4, 2),
        'M_R_3': (4, 3),
        'M_R_4': (4, 4),
        
        # Target nodes
        'T_0': (5, 0),
        'T_1': (5, 1),
        'T_2': (5, 2),
        'T_3': (5, 3),
        'T_4': (5, 4),
    }
}

# Configuration 6: Grid Network with Full Bridges
CONFIG_6 = {
    'name': 'Grid Network with Full Bridges',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', float('inf')),
        ('Super_S', 'S_1', float('inf')),
        ('Super_S', 'S_2', float('inf')),
        ('Super_S', 'S_3', float('inf')),
        ('Super_S', 'S_4', float('inf')),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', float('inf')),
        ('T_1', 'Super_T', float('inf')),
        ('T_2', 'Super_T', float('inf')),
        ('T_3', 'Super_T', float('inf')),
        ('T_4', 'Super_T', float('inf')),
        
        # Connect source nodes to left structure
        ('S_0', 'M_L_0', float('inf')),
        ('S_1', 'M_L_1', float('inf')),
        ('S_2', 'M_L_2', float('inf')),
        ('S_3', 'M_L_3', float('inf')),
        ('S_4', 'M_L_4', float('inf')),
        
        # Connect left structure nodes vertically
        ('M_L_0', 'M_L_1', float('inf')),
        ('M_L_1', 'M_L_2', float('inf')),
        ('M_L_2', 'M_L_3', float('inf')),
        ('M_L_3', 'M_L_4', float('inf')),
        
        # Connect right structure nodes vertically
        ('M_R_0', 'M_R_1', float('inf')),
        ('M_R_1', 'M_R_2', float('inf')),
        ('M_R_2', 'M_R_3', float('inf')),
        ('M_R_3', 'M_R_4', float('inf')),
        
        # Connect bridge nodes in column 2 vertically
        ('B_0_0', 'B_0_1', float('inf')),
        ('B_0_1', 'B_0_2', float('inf')),
        ('B_0_2', 'B_0_3', float('inf')),
        ('B_0_3', 'B_0_4', float('inf')),
        
        # Connect bridge nodes in column 3 vertically
        ('B_1_0', 'B_1_1', float('inf')),
        ('B_1_1', 'B_1_2', float('inf')),
        ('B_1_2', 'B_1_3', float('inf')),
        ('B_1_3', 'B_1_4', float('inf')),
        
        # Connect left structure to column 2 bridge nodes
        ('M_L_0', 'B_0_0', float('inf')),
        ('M_L_1', 'B_0_1', float('inf')),
        ('M_L_2', 'B_0_2', float('inf')),
        ('M_L_3', 'B_0_3', float('inf')),
        ('M_L_4', 'B_0_4', float('inf')),
        
        # Connect column 2 bridge nodes to column 3 bridge nodes
        ('B_0_0', 'B_1_0', float('inf')),
        ('B_0_1', 'B_1_1', float('inf')),
        ('B_0_2', 'B_1_2', float('inf')),
        ('B_0_3', 'B_1_3', float('inf')),
        ('B_0_4', 'B_1_4', float('inf')),
        
        # Connect column 3 bridge nodes to right structure
        ('B_1_0', 'M_R_0', float('inf')),
        ('B_1_1', 'M_R_1', float('inf')),
        ('B_1_2', 'M_R_2', float('inf')),
        ('B_1_3', 'M_R_3', float('inf')),
        ('B_1_4', 'M_R_4', float('inf')),
        
        # Connect right structure to targets
        ('M_R_0', 'T_0', float('inf')),
        ('M_R_1', 'T_1', float('inf')),
        ('M_R_2', 'T_2', float('inf')),
        ('M_R_3', 'T_3', float('inf')),
        ('M_R_4', 'T_4', float('inf')),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (6, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Bridge nodes in column 2
        'B_0_0': (2, 0),
        'B_0_1': (2, 1),
        'B_0_2': (2, 2),
        'B_0_3': (2, 3),
        'B_0_4': (2, 4),
        
        # Bridge nodes in column 3
        'B_1_0': (3, 0),
        'B_1_1': (3, 1),
        'B_1_2': (3, 2),
        'B_1_3': (3, 3),
        'B_1_4': (3, 4),
        
        # Right structure nodes
        'M_R_0': (4, 0),
        'M_R_1': (4, 1),
        'M_R_2': (4, 2),
        'M_R_3': (4, 3),
        'M_R_4': (4, 4),
        
        # Target nodes
        'T_0': (5, 0),
        'T_1': (5, 1),
        'T_2': (5, 2),
        'T_3': (5, 3),
        'T_4': (5, 4),
    }
}

# Configuration 7: Fully Connected Grid Network
CONFIG_7 = {
    'name': 'Fully Connected Grid Network',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === COLUMN 0 (SOURCE NODES) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # === COLUMN 1 (LEFT STRUCTURE) ===
        # Left structure nodes to source nodes (horizontal)
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Left structure diagonal connections to source nodes
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # === COLUMN 2 (BRIDGE NODES COL 1) ===
        # Left structure to bridge column 1 (horizontal)
        ('M_L_0', 'B_0_0', 20),
        ('M_L_1', 'B_0_1', 20),
        ('M_L_2', 'B_0_2', 20),
        ('M_L_3', 'B_0_3', 20),
        ('M_L_4', 'B_0_4', 20),
        
        # Diagonal connections from left structure to bridge column 1
        ('M_L_0', 'B_0_1', 20),
        ('M_L_1', 'B_0_0', 20),
        ('M_L_1', 'B_0_2', 20),
        ('M_L_2', 'B_0_1', 20),
        ('M_L_2', 'B_0_3', 20),
        ('M_L_3', 'B_0_2', 20),
        ('M_L_3', 'B_0_4', 20),
        ('M_L_4', 'B_0_3', 20),
        
        # Vertical connections in bridge column 1
        ('B_0_0', 'B_0_1', 20),
        ('B_0_1', 'B_0_2', 20),
        ('B_0_2', 'B_0_3', 20),
        ('B_0_3', 'B_0_4', 20),
        
        # === COLUMN 3 (BRIDGE NODES COL 2) ===
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_0', 'B_1_0', 20),
        ('B_0_1', 'B_1_1', 20),
        ('B_0_2', 'B_1_2', 20),
        ('B_0_3', 'B_1_3', 20),
        ('B_0_4', 'B_1_4', 20),
        
        # Diagonal connections from bridge column 1 to bridge column 2
        ('B_0_0', 'B_1_1', 20),
        ('B_0_1', 'B_1_0', 20),
        ('B_0_1', 'B_1_2', 20),
        ('B_0_2', 'B_1_1', 20),
        ('B_0_2', 'B_1_3', 20),
        ('B_0_3', 'B_1_2', 20),
        ('B_0_3', 'B_1_4', 20),
        ('B_0_4', 'B_1_3', 20),
        
        # Vertical connections in bridge column 2
        ('B_1_0', 'B_1_1', 20),
        ('B_1_1', 'B_1_2', 20),
        ('B_1_2', 'B_1_3', 20),
        ('B_1_3', 'B_1_4', 20),
        
        # === COLUMN 4 (RIGHT STRUCTURE) ===
        # Bridge column 2 to right structure (horizontal)
        ('B_1_0', 'M_R_0', 20),
        ('B_1_1', 'M_R_1', 20),
        ('B_1_2', 'M_R_2', 20),
        ('B_1_3', 'M_R_3', 20),
        ('B_1_4', 'M_R_4', 20),
        
        # Diagonal connections from bridge column 2 to right structure
        ('B_1_0', 'M_R_1', 20),
        ('B_1_1', 'M_R_0', 20),
        ('B_1_1', 'M_R_2', 20),
        ('B_1_2', 'M_R_1', 20),
        ('B_1_2', 'M_R_3', 20),
        ('B_1_3', 'M_R_2', 20),
        ('B_1_3', 'M_R_4', 20),
        ('B_1_4', 'M_R_3', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # === COLUMN 5 (TARGET NODES) ===
        # Right structure to target nodes (horizontal)
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections from right structure to target nodes
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (6, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Bridge nodes in column 2
        'B_0_0': (2, 0),
        'B_0_1': (2, 1),
        'B_0_2': (2, 2),
        'B_0_3': (2, 3),
        'B_0_4': (2, 4),
        
        # Bridge nodes in column 3
        'B_1_0': (3, 0),
        'B_1_1': (3, 1),
        'B_1_2': (3, 2),
        'B_1_3': (3, 3),
        'B_1_4': (3, 4),
        
        # Right structure nodes
        'M_R_0': (4, 0),
        'M_R_1': (4, 1),
        'M_R_2': (4, 2),
        'M_R_3': (4, 3),
        'M_R_4': (4, 4),
        
        # Target nodes
        'T_0': (5, 0),
        'T_1': (5, 1),
        'T_2': (5, 2),
        'T_3': (5, 3),
        'T_4': (5, 4),
    }
}

# Configuration 8: Fully Connected Grid Network with Diagonal Connections
CONFIG_8 = {
    'name': 'Fully Connected Grid Network with Diagonal Connections',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === COLUMN 0 (SOURCE NODES) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # === COLUMN 1 (LEFT STRUCTURE) ===
        # Left structure nodes to source nodes (horizontal)
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Left structure diagonal connections to source nodes
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # === COLUMN 2 (BRIDGE NODES COL 1) ===
        # Left structure to bridge column 1 (horizontal)
        ('M_L_1', 'B_0_1', 20),
        ('M_L_2', 'B_0_2', 20),
        ('M_L_3', 'B_0_3', 20),
        
        # Diagonal connections from left structure to bridge column 1
        ('M_L_0', 'B_0_1', 20),
        ('M_L_1', 'B_0_2', 20),
        ('M_L_2', 'B_0_1', 20),
        ('M_L_2', 'B_0_3', 20),
        ('M_L_3', 'B_0_2', 20),
        ('M_L_4', 'B_0_3', 20),
        
        # Vertical connections in bridge column 1
        ('B_0_1', 'B_0_2', 20),
        ('B_0_2', 'B_0_3', 20),
        
        # === COLUMN 3 (BRIDGE NODES COL 2) ===
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_1', 'B_1_1', 20),
        ('B_0_2', 'B_1_2', 20),
        ('B_0_3', 'B_1_3', 20),
        
        # Diagonal connections from bridge column 1 to bridge column 2
        ('B_0_1', 'B_1_2', 20),
        ('B_0_2', 'B_1_1', 20),
        ('B_0_2', 'B_1_3', 20),
        ('B_0_3', 'B_1_2', 20),
        
        # Vertical connections in bridge column 2
        ('B_1_1', 'B_1_2', 20),
        ('B_1_2', 'B_1_3', 20),
        
        # === COLUMN 4 (RIGHT STRUCTURE) ===
        # Bridge column 2 to right structure (horizontal)
        ('B_1_1', 'M_R_1', 20),
        ('B_1_2', 'M_R_2', 20),
        ('B_1_3', 'M_R_3', 20),
        
        # Diagonal connections from bridge column 2 to right structure
        ('B_1_1', 'M_R_0', 20),
        ('B_1_1', 'M_R_2', 20),
        ('B_1_2', 'M_R_1', 20),
        ('B_1_2', 'M_R_3', 20),
        ('B_1_3', 'M_R_2', 20),
        ('B_1_3', 'M_R_4', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # === COLUMN 5 (TARGET NODES) ===
        # Right structure to target nodes (horizontal)
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections from right structure to target nodes
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (6, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Bridge nodes in column 2
        'B_0_1': (2, 1),
        'B_0_2': (2, 2),
        'B_0_3': (2, 3),
        
        # Bridge nodes in column 3
        'B_1_1': (3, 1),
        'B_1_2': (3, 2),
        'B_1_3': (3, 3),
        
        # Right structure nodes
        'M_R_0': (4, 0),
        'M_R_1': (4, 1),
        'M_R_2': (4, 2),
        'M_R_3': (4, 3),
        'M_R_4': (4, 4),
        
        # Target nodes
        'T_0': (5, 0),
        'T_1': (5, 1),
        'T_2': (5, 2),
        'T_3': (5, 3),
        'T_4': (5, 4),
    }
}

# Configuration 9: Fully Connected Grid Network with Bridges
CONFIG_9 = {
    'name': 'Fully Connected Grid Network with Bridges',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === COLUMN 0 (SOURCE NODES) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # === COLUMN 1 (LEFT STRUCTURE) ===
        # Left structure nodes to source nodes (horizontal)
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Left structure diagonal connections to source nodes
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # === COLUMN 2 (BRIDGE NODES COL 1) ===
        # Left structure to bridge column 1 (horizontal)
        ('M_L_2', 'B_0_2', 20),
        
        # Diagonal connections from left structure to bridge column 1
        ('M_L_1', 'B_0_2', 20),
        ('M_L_3', 'B_0_2', 20),
        
        # Vertical connections in bridge column 1
        
        # === COLUMN 3 (BRIDGE NODES COL 2) ===
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_2', 'B_1_2', 20),
        
        # Diagonal connections from bridge column 1 to bridge column 2
        
        # Vertical connections in bridge column 2
        
        # === COLUMN 4 (RIGHT STRUCTURE) ===
        # Bridge column 2 to right structure (horizontal)
        ('B_1_2', 'M_R_2', 20),
        
        # Diagonal connections from bridge column 2 to right structure
        ('B_1_2', 'M_R_1', 20),
        ('B_1_2', 'M_R_3', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # === COLUMN 5 (TARGET NODES) ===
        # Right structure to target nodes (horizontal)
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections from right structure to target nodes
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (6, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Bridge nodes in column 2
        'B_0_2': (2, 2),
        
        # Bridge nodes in column 3
        'B_1_2': (3, 2),
        
        # Right structure nodes
        'M_R_0': (4, 0),
        'M_R_1': (4, 1),
        'M_R_2': (4, 2),
        'M_R_3': (4, 3),
        'M_R_4': (4, 4),
        
        # Target nodes
        'T_0': (5, 0),
        'T_1': (5, 1),
        'T_2': (5, 2),
        'T_3': (5, 3),
        'T_4': (5, 4),
    }
}

# Configuration 10: Extended Grid Network with Additional Structure Columns
CONFIG_10 = {
    'name': 'Extended Grid Network',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === COLUMN 0 (SOURCE NODES) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # === COLUMN 1 (LEFT STRUCTURE) ===
        # Left structure nodes to source nodes (horizontal)
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Left structure diagonal connections to source nodes
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # === COLUMN 2 (ADDITIONAL LEFT STRUCTURE) ===
        # Left structure to additional left structure (horizontal)
        ('M_L_0', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_3', 20),
        ('M_L_4', 'M_L2_4', 20),
        
        # Diagonal connections to additional left structure
        ('M_L_0', 'M_L2_1', 20),
        ('M_L_1', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_2', 20),
        ('M_L_2', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_3', 20),
        ('M_L_3', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_4', 20),
        ('M_L_4', 'M_L2_3', 20),
        
        # Vertical connections in additional left structure
        ('M_L2_0', 'M_L2_1', 20),
        ('M_L2_1', 'M_L2_2', 20),
        ('M_L2_2', 'M_L2_3', 20),
        ('M_L2_3', 'M_L2_4', 20),
        
        # === COLUMN 3 (BRIDGE NODES COL 1) ===
        # Additional left structure to bridge column 1
        ('M_L2_2', 'B_0_2', 20),
        
        # Diagonal connections to bridge column 1
        ('M_L2_1', 'B_0_2', 20),
        ('M_L2_3', 'B_0_2', 20),
        
        # === COLUMN 4 (BRIDGE NODES COL 2) ===
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_2', 'B_1_2', 50),
        
        # === COLUMN 5 (ADDITIONAL RIGHT STRUCTURE) ===
        # Bridge column 2 to additional right structure
        ('B_1_2', 'M_R2_2', 20),
        
        # Diagonal connections to additional right structure
        ('B_1_2', 'M_R2_1', 20),
        ('B_1_2', 'M_R2_3', 20),
        
        # Vertical connections in additional right structure
        ('M_R2_0', 'M_R2_1', 20),
        ('M_R2_1', 'M_R2_2', 20),
        ('M_R2_2', 'M_R2_3', 20),
        ('M_R2_3', 'M_R2_4', 20),
        
        # === COLUMN 6 (RIGHT STRUCTURE) ===
        # Additional right structure to right structure
        ('M_R2_0', 'M_R_0', 20),
        ('M_R2_1', 'M_R_1', 20),
        ('M_R2_2', 'M_R_2', 20),
        ('M_R2_3', 'M_R_3', 20),
        ('M_R2_4', 'M_R_4', 20),
        
        # Diagonal connections to right structure
        ('M_R2_0', 'M_R_1', 20),
        ('M_R2_1', 'M_R_0', 20),
        ('M_R2_1', 'M_R_2', 20),
        ('M_R2_2', 'M_R_1', 20),
        ('M_R2_2', 'M_R_3', 20),
        ('M_R2_3', 'M_R_2', 20),
        ('M_R2_3', 'M_R_4', 20),
        ('M_R2_4', 'M_R_3', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # === COLUMN 7 (TARGET NODES) ===
        # Right structure to target nodes (horizontal)
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections from right structure to target nodes
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (8, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Additional left structure nodes
        'M_L2_0': (2, 0),
        'M_L2_1': (2, 1),
        'M_L2_2': (2, 2),
        'M_L2_3': (2, 3),
        'M_L2_4': (2, 4),
        
        # Bridge nodes in column 3
        'B_0_2': (3, 2),
        
        # Bridge nodes in column 4
        'B_1_2': (4, 2),
        
        # Additional right structure nodes
        'M_R2_0': (5, 0),
        'M_R2_1': (5, 1),
        'M_R2_2': (5, 2),
        'M_R2_3': (5, 3),
        'M_R2_4': (5, 4),
        
        # Right structure nodes
        'M_R_0': (6, 0),
        'M_R_1': (6, 1),
        'M_R_2': (6, 2),
        'M_R_3': (6, 3),
        'M_R_4': (6, 4),
        
        # Target nodes
        'T_0': (7, 0),
        'T_1': (7, 1),
        'T_2': (7, 2),
        'T_3': (7, 3),
        'T_4': (7, 4),
    }
}

# Configuration 11: Extended Grid Network with Two Additional Structure Columns on Each Side
CONFIG_11 = {
    'name': 'Extended Grid Network with Multiple Columns',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === COLUMN 0 (SOURCE NODES) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # === COLUMN 1 (LEFT STRUCTURE) ===
        # Left structure nodes to source nodes (horizontal)
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Left structure diagonal connections to source nodes
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # === COLUMN 2 (ADDITIONAL LEFT STRUCTURE 1) ===
        # Left structure to additional left structure 1 (horizontal)
        ('M_L_0', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_3', 20),
        ('M_L_4', 'M_L2_4', 20),
        
        # Diagonal connections to additional left structure 1
        ('M_L_0', 'M_L2_1', 20),
        ('M_L_1', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_2', 20),
        ('M_L_2', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_3', 20),
        ('M_L_3', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_4', 20),
        ('M_L_4', 'M_L2_3', 20),
        
        # Vertical connections in additional left structure 1
        ('M_L2_0', 'M_L2_1', 20),
        ('M_L2_1', 'M_L2_2', 20),
        ('M_L2_2', 'M_L2_3', 20),
        ('M_L2_3', 'M_L2_4', 20),
        
        # === COLUMN 3 (ADDITIONAL LEFT STRUCTURE 2) ===
        # Additional left structure 1 to additional left structure 2 (horizontal)
        ('M_L2_0', 'M_L3_0', 20),
        ('M_L2_1', 'M_L3_1', 20),
        ('M_L2_2', 'M_L3_2', 20),
        ('M_L2_3', 'M_L3_3', 20),
        ('M_L2_4', 'M_L3_4', 20),
        
        # Diagonal connections to additional left structure 2
        ('M_L2_0', 'M_L3_1', 20),
        ('M_L2_1', 'M_L3_0', 20),
        ('M_L2_1', 'M_L3_2', 20),
        ('M_L2_2', 'M_L3_1', 20),
        ('M_L2_2', 'M_L3_3', 20),
        ('M_L2_3', 'M_L3_2', 20),
        ('M_L2_3', 'M_L3_4', 20),
        ('M_L2_4', 'M_L3_3', 20),
        
        # Vertical connections in additional left structure 2
        ('M_L3_0', 'M_L3_1', 20),
        ('M_L3_1', 'M_L3_2', 20),
        ('M_L3_2', 'M_L3_3', 20),
        ('M_L3_3', 'M_L3_4', 20),
        
        # === COLUMN 4 (BRIDGE NODES COL 1) ===
        # Additional left structure 2 to bridge column 1
        ('M_L3_2', 'B_0_2', 20),
        
        # Diagonal connections to bridge column 1
        ('M_L3_1', 'B_0_2', 20),
        ('M_L3_3', 'B_0_2', 20),
        
        # === COLUMN 5 (BRIDGE NODES COL 2) ===
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_2', 'B_1_2', 50),
        
        # === COLUMN 6 (ADDITIONAL RIGHT STRUCTURE 1) ===
        # Bridge column 2 to additional right structure 1
        ('B_1_2', 'M_R2_2', 20),
        
        # Diagonal connections to additional right structure 1
        ('B_1_2', 'M_R2_1', 20),
        ('B_1_2', 'M_R2_3', 20),
        
        # Vertical connections in additional right structure 1
        ('M_R2_0', 'M_R2_1', 20),
        ('M_R2_1', 'M_R2_2', 20),
        ('M_R2_2', 'M_R2_3', 20),
        ('M_R2_3', 'M_R2_4', 20),
        
        # === COLUMN 7 (ADDITIONAL RIGHT STRUCTURE 2) ===
        # Additional right structure 1 to additional right structure 2
        ('M_R2_0', 'M_R3_0', 20),
        ('M_R2_1', 'M_R3_1', 20),
        ('M_R2_2', 'M_R3_2', 20),
        ('M_R2_3', 'M_R3_3', 20),
        ('M_R2_4', 'M_R3_4', 20),
        
        # Diagonal connections to additional right structure 2
        ('M_R2_0', 'M_R3_1', 20),
        ('M_R2_1', 'M_R3_0', 20),
        ('M_R2_1', 'M_R3_2', 20),
        ('M_R2_2', 'M_R3_1', 20),
        ('M_R2_2', 'M_R3_3', 20),
        ('M_R2_3', 'M_R3_2', 20),
        ('M_R2_3', 'M_R3_4', 20),
        ('M_R2_4', 'M_R3_3', 20),
        
        # Vertical connections in additional right structure 2
        ('M_R3_0', 'M_R3_1', 20),
        ('M_R3_1', 'M_R3_2', 20),
        ('M_R3_2', 'M_R3_3', 20),
        ('M_R3_3', 'M_R3_4', 20),
        
        # === COLUMN 8 (RIGHT STRUCTURE) ===
        # Additional right structure 2 to right structure
        ('M_R3_0', 'M_R_0', 20),
        ('M_R3_1', 'M_R_1', 20),
        ('M_R3_2', 'M_R_2', 20),
        ('M_R3_3', 'M_R_3', 20),
        ('M_R3_4', 'M_R_4', 20),
        
        # Diagonal connections to right structure
        ('M_R3_0', 'M_R_1', 20),
        ('M_R3_1', 'M_R_0', 20),
        ('M_R3_1', 'M_R_2', 20),
        ('M_R3_2', 'M_R_1', 20),
        ('M_R3_2', 'M_R_3', 20),
        ('M_R3_3', 'M_R_2', 20),
        ('M_R3_3', 'M_R_4', 20),
        ('M_R3_4', 'M_R_3', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # === COLUMN 9 (TARGET NODES) ===
        # Right structure to target nodes (horizontal)
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections from right structure to target nodes
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (10, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Additional left structure nodes 1
        'M_L2_0': (2, 0),
        'M_L2_1': (2, 1),
        'M_L2_2': (2, 2),
        'M_L2_3': (2, 3),
        'M_L2_4': (2, 4),
        
        # Additional left structure nodes 2
        'M_L3_0': (3, 0),
        'M_L3_1': (3, 1),
        'M_L3_2': (3, 2),
        'M_L3_3': (3, 3),
        'M_L3_4': (3, 4),
        
        # Bridge nodes in column 4
        'B_0_2': (4, 2),
        
        # Bridge nodes in column 5
        'B_1_2': (5, 2),
        
        # Additional right structure nodes 1
        'M_R2_0': (6, 0),
        'M_R2_1': (6, 1),
        'M_R2_2': (6, 2),
        'M_R2_3': (6, 3),
        'M_R2_4': (6, 4),
        
        # Additional right structure nodes 2
        'M_R3_0': (7, 0),
        'M_R3_1': (7, 1),
        'M_R3_2': (7, 2),
        'M_R3_3': (7, 3),
        'M_R3_4': (7, 4),
        
        # Right structure nodes
        'M_R_0': (8, 0),
        'M_R_1': (8, 1),
        'M_R_2': (8, 2),
        'M_R_3': (8, 3),
        'M_R_4': (8, 4),
        
        # Target nodes
        'T_0': (9, 0),
        'T_1': (9, 1),
        'T_2': (9, 2),
        'T_3': (9, 3),
        'T_4': (9, 4),
    }
}

# Configuration 12: Extended Grid Network with Full Bridge Connections
CONFIG_12 = {
    'name': 'Extended Grid Network with Full Bridges',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 10),
        ('Super_S', 'S_1', 10),
        ('Super_S', 'S_2', 10),
        ('Super_S', 'S_3', 10),
        ('Super_S', 'S_4', 10),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 50),
        ('T_1', 'Super_T', 50),
        ('T_2', 'Super_T', 50),
        ('T_3', 'Super_T', 50),
        ('T_4', 'Super_T', 50),
        
        # === SOURCE AND LEFT STRUCTURE CONNECTIONS (SAME AS CONFIG_11) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 20),
        ('S_1', 'S_2', 20),
        ('S_2', 'S_3', 20),
        ('S_3', 'S_4', 20),
        
        # Source to left structure
        ('S_0', 'M_L_0', 20),
        ('S_1', 'M_L_1', 20),
        ('S_2', 'M_L_2', 20),
        ('S_3', 'M_L_3', 20),
        ('S_4', 'M_L_4', 20),
        
        # Diagonal connections 
        ('S_0', 'M_L_1', 20),
        ('S_1', 'M_L_0', 20),
        ('S_1', 'M_L_2', 20),
        ('S_2', 'M_L_1', 20),
        ('S_2', 'M_L_3', 20),
        ('S_3', 'M_L_2', 20),
        ('S_3', 'M_L_4', 20),
        ('S_4', 'M_L_3', 20),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 20),
        ('M_L_1', 'M_L_2', 20),
        ('M_L_2', 'M_L_3', 20),
        ('M_L_3', 'M_L_4', 20),
        
        # Left structure to additional left structure 1
        ('M_L_0', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_3', 20),
        ('M_L_4', 'M_L2_4', 20),
        
        # Diagonal connections
        ('M_L_0', 'M_L2_1', 20),
        ('M_L_1', 'M_L2_0', 20),
        ('M_L_1', 'M_L2_2', 20),
        ('M_L_2', 'M_L2_1', 20),
        ('M_L_2', 'M_L2_3', 20),
        ('M_L_3', 'M_L2_2', 20),
        ('M_L_3', 'M_L2_4', 20),
        ('M_L_4', 'M_L2_3', 20),
        
        # Vertical connections in additional left structure 1
        ('M_L2_0', 'M_L2_1', 20),
        ('M_L2_1', 'M_L2_2', 20),
        ('M_L2_2', 'M_L2_3', 20),
        ('M_L2_3', 'M_L2_4', 20),
        
        # Additional left structure 1 to additional left structure 2
        ('M_L2_0', 'M_L3_0', 20),
        ('M_L2_1', 'M_L3_1', 20),
        ('M_L2_2', 'M_L3_2', 20),
        ('M_L2_3', 'M_L3_3', 20),
        ('M_L2_4', 'M_L3_4', 20),
        
        # Diagonal connections
        ('M_L2_0', 'M_L3_1', 20),
        ('M_L2_1', 'M_L3_0', 20),
        ('M_L2_1', 'M_L3_2', 20),
        ('M_L2_2', 'M_L3_1', 20),
        ('M_L2_2', 'M_L3_3', 20),
        ('M_L2_3', 'M_L3_2', 20),
        ('M_L2_3', 'M_L3_4', 20),
        ('M_L2_4', 'M_L3_3', 20),
        
        # Vertical connections in additional left structure 2
        ('M_L3_0', 'M_L3_1', 20),
        ('M_L3_1', 'M_L3_2', 20),
        ('M_L3_2', 'M_L3_3', 20),
        ('M_L3_3', 'M_L3_4', 20),
        
        # === BRIDGE CONNECTIONS (EXPANDED FOR ALL ROWS) ===
        # Additional left structure 2 to bridge column 1 (horizontal)
        ('M_L3_0', 'B_0_0', 20),
        ('M_L3_1', 'B_0_1', 20),
        ('M_L3_2', 'B_0_2', 20),
        ('M_L3_3', 'B_0_3', 20),
        ('M_L3_4', 'B_0_4', 20),
        
        # Diagonal connections to bridge column 1
        ('M_L3_0', 'B_0_1', 20),
        ('M_L3_1', 'B_0_0', 20),
        ('M_L3_1', 'B_0_2', 20),
        ('M_L3_2', 'B_0_1', 20),
        ('M_L3_2', 'B_0_3', 20),
        ('M_L3_3', 'B_0_2', 20),
        ('M_L3_3', 'B_0_4', 20),
        ('M_L3_4', 'B_0_3', 20),
        
        # Vertical connections in bridge column 1
        ('B_0_0', 'B_0_1', 50),
        ('B_0_1', 'B_0_2', 50),
        ('B_0_2', 'B_0_3', 50),
        ('B_0_3', 'B_0_4', 50),
        
        # Bridge column 1 to bridge column 2 (horizontal)
        ('B_0_0', 'B_1_0', 50),
        ('B_0_1', 'B_1_1', 50),
        ('B_0_2', 'B_1_2', 50),  # Middle bridge has higher capacity
        ('B_0_3', 'B_1_3', 50),
        ('B_0_4', 'B_1_4', 50),
        
        # Diagonal connections between bridge columns
        ('B_0_0', 'B_1_1', 50),
        ('B_0_1', 'B_1_0', 50),
        ('B_0_1', 'B_1_2', 50),
        ('B_0_2', 'B_1_1', 50),
        ('B_0_2', 'B_1_3', 50),
        ('B_0_3', 'B_1_2', 50),
        ('B_0_3', 'B_1_4', 50),
        ('B_0_4', 'B_1_3', 50),
        
        # Vertical connections in bridge column 2
        ('B_1_0', 'B_1_1', 50),
        ('B_1_1', 'B_1_2', 50),
        ('B_1_2', 'B_1_3', 50),
        ('B_1_3', 'B_1_4', 50),
        
        # Bridge column 2 to additional right structure 1 (horizontal)
        ('B_1_0', 'M_R2_0', 20),
        ('B_1_1', 'M_R2_1', 20),
        ('B_1_2', 'M_R2_2', 20),
        ('B_1_3', 'M_R2_3', 20),
        ('B_1_4', 'M_R2_4', 20),
        
        # Diagonal connections to additional right structure 1
        ('B_1_0', 'M_R2_1', 20),
        ('B_1_1', 'M_R2_0', 20),
        ('B_1_1', 'M_R2_2', 20),
        ('B_1_2', 'M_R2_1', 20),
        ('B_1_2', 'M_R2_3', 20),
        ('B_1_3', 'M_R2_2', 20),
        ('B_1_3', 'M_R2_4', 20),
        ('B_1_4', 'M_R2_3', 20),
        
        # === RIGHT STRUCTURE CONNECTIONS (SAME AS CONFIG_11) ===
        # Vertical connections in additional right structure 1
        ('M_R2_0', 'M_R2_1', 20),
        ('M_R2_1', 'M_R2_2', 20),
        ('M_R2_2', 'M_R2_3', 20),
        ('M_R2_3', 'M_R2_4', 20),
        
        # Additional right structure 1 to additional right structure 2
        ('M_R2_0', 'M_R3_0', 20),
        ('M_R2_1', 'M_R3_1', 20),
        ('M_R2_2', 'M_R3_2', 20),
        ('M_R2_3', 'M_R3_3', 20),
        ('M_R2_4', 'M_R3_4', 20),
        
        # Diagonal connections
        ('M_R2_0', 'M_R3_1', 20),
        ('M_R2_1', 'M_R3_0', 20),
        ('M_R2_1', 'M_R3_2', 20),
        ('M_R2_2', 'M_R3_1', 20),
        ('M_R2_2', 'M_R3_3', 20),
        ('M_R2_3', 'M_R3_2', 20),
        ('M_R2_3', 'M_R3_4', 20),
        ('M_R2_4', 'M_R3_3', 20),
        
        # Vertical connections in additional right structure 2
        ('M_R3_0', 'M_R3_1', 20),
        ('M_R3_1', 'M_R3_2', 20),
        ('M_R3_2', 'M_R3_3', 20),
        ('M_R3_3', 'M_R3_4', 20),
        
        # Additional right structure 2 to right structure
        ('M_R3_0', 'M_R_0', 20),
        ('M_R3_1', 'M_R_1', 20),
        ('M_R3_2', 'M_R_2', 20),
        ('M_R3_3', 'M_R_3', 20),
        ('M_R3_4', 'M_R_4', 20),
        
        # Diagonal connections
        ('M_R3_0', 'M_R_1', 20),
        ('M_R3_1', 'M_R_0', 20),
        ('M_R3_1', 'M_R_2', 20),
        ('M_R3_2', 'M_R_1', 20),
        ('M_R3_2', 'M_R_3', 20),
        ('M_R3_3', 'M_R_2', 20),
        ('M_R3_3', 'M_R_4', 20),
        ('M_R3_4', 'M_R_3', 20),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 20),
        ('M_R_1', 'M_R_2', 20),
        ('M_R_2', 'M_R_3', 20),
        ('M_R_3', 'M_R_4', 20),
        
        # Right structure to target nodes
        ('M_R_0', 'T_0', 20),
        ('M_R_1', 'T_1', 20),
        ('M_R_2', 'T_2', 20),
        ('M_R_3', 'T_3', 20),
        ('M_R_4', 'T_4', 20),
        
        # Diagonal connections
        ('M_R_0', 'T_1', 20),
        ('M_R_1', 'T_0', 20),
        ('M_R_1', 'T_2', 20),
        ('M_R_2', 'T_1', 20),
        ('M_R_2', 'T_3', 20),
        ('M_R_3', 'T_2', 20),
        ('M_R_3', 'T_4', 20),
        ('M_R_4', 'T_3', 20),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 20),
        ('T_1', 'T_2', 20),
        ('T_2', 'T_3', 20),
        ('T_3', 'T_4', 20),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (10, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Additional left structure nodes 1
        'M_L2_0': (2, 0),
        'M_L2_1': (2, 1),
        'M_L2_2': (2, 2),
        'M_L2_3': (2, 3),
        'M_L2_4': (2, 4),
        
        # Additional left structure nodes 2
        'M_L3_0': (3, 0),
        'M_L3_1': (3, 1),
        'M_L3_2': (3, 2),
        'M_L3_3': (3, 3),
        'M_L3_4': (3, 4),
        
        # Bridge nodes in column 4 (all rows)
        'B_0_0': (4, 0),
        'B_0_1': (4, 1),
        'B_0_2': (4, 2),
        'B_0_3': (4, 3),
        'B_0_4': (4, 4),
        
        # Bridge nodes in column 5 (all rows)
        'B_1_0': (5, 0),
        'B_1_1': (5, 1),
        'B_1_2': (5, 2),
        'B_1_3': (5, 3),
        'B_1_4': (5, 4),
        
        # Additional right structure nodes 1
        'M_R2_0': (6, 0),
        'M_R2_1': (6, 1),
        'M_R2_2': (6, 2),
        'M_R2_3': (6, 3),
        'M_R2_4': (6, 4),
        
        # Additional right structure nodes 2
        'M_R3_0': (7, 0),
        'M_R3_1': (7, 1),
        'M_R3_2': (7, 2),
        'M_R3_3': (7, 3),
        'M_R3_4': (7, 4),
        
        # Right structure nodes
        'M_R_0': (8, 0),
        'M_R_1': (8, 1),
        'M_R_2': (8, 2),
        'M_R_3': (8, 3),
        'M_R_4': (8, 4),
        
        # Target nodes
        'T_0': (9, 0),
        'T_1': (9, 1),
        'T_2': (9, 2),
        'T_3': (9, 3),
        'T_4': (9, 4),
    }
}

# Configuration 13: Grid Network with Full Bridges and Full Forward and Backward Connections
CONFIG_13 = {
    'name': 'Extended Grid Network with Full Bridges',
    'edges': [
        # Connect super source to all source nodes
        ('Super_S', 'S_0', 1),
        ('Super_S', 'S_1', 1),
        ('Super_S', 'S_2', 1),
        ('Super_S', 'S_3', 1),
        ('Super_S', 'S_4', 1),
        
        # Connect all target nodes to super sink
        ('T_0', 'Super_T', 1),
        ('T_1', 'Super_T', 1),
        ('T_2', 'Super_T', 1),
        ('T_3', 'Super_T', 1),
        ('T_4', 'Super_T', 1),
        
        # === SOURCE AND LEFT STRUCTURE CONNECTIONS (SAME AS CONFIG_11) ===
        # Vertical connections between source nodes
        ('S_0', 'S_1', 5),
        ('S_1', 'S_2', 5),
        ('S_2', 'S_3', 5),
        ('S_3', 'S_4', 5),
        
        # Source to left structure
        ('S_0', 'M_L_0', 5),
        ('S_1', 'M_L_1', 5),
        ('S_2', 'M_L_2', 5),
        ('S_3', 'M_L_3', 5),
        ('S_4', 'M_L_4', 5),

        # Left structure to source nodes
        ('M_L_0', 'S_0', 5),
        ('M_L_1', 'S_1', 5),
        ('M_L_2', 'S_2', 5),
        ('M_L_3', 'S_3', 5),
        ('M_L_4', 'S_4', 5),
        
        # Diagonal connections 
        ('S_0', 'M_L_1', 5),
        ('S_1', 'M_L_0', 5),
        ('S_1', 'M_L_2', 5),
        ('S_2', 'M_L_1', 5),
        ('S_2', 'M_L_3', 5),
        ('S_3', 'M_L_2', 5),
        ('S_3', 'M_L_4', 5),
        ('S_4', 'M_L_3', 5),

        # Backward diagonal connections
        ('M_L_0', 'S_1', 5),
        ('M_L_1', 'S_0', 5),
        ('M_L_1', 'S_2', 5),
        ('M_L_2', 'S_1', 5),
        ('M_L_2', 'S_3', 5),
        ('M_L_3', 'S_2', 5),
        ('M_L_3', 'S_4', 5),
        ('M_L_4', 'S_3', 5),
        
        # Vertical connections in left structure
        ('M_L_0', 'M_L_1', 5),
        ('M_L_1', 'M_L_2', 5),
        ('M_L_2', 'M_L_3', 5),
        ('M_L_3', 'M_L_4', 5),

        # Downward connections in left structure
        ('M_L_1', 'M_L_0', 5),
        ('M_L_2', 'M_L_1', 5),
        ('M_L_3', 'M_L_2', 5),
        ('M_L_4', 'M_L_3', 5),
      
        # Left structure to additional left structure 1
        ('M_L_0', 'M_L2_0', 5),
        ('M_L_1', 'M_L2_1', 5),
        ('M_L_2', 'M_L2_2', 5),
        ('M_L_3', 'M_L2_3', 5),
        ('M_L_4', 'M_L2_4', 5),

        # Additional left structure 1 to left structure
        ('M_L2_0', 'M_L_0', 5),
        ('M_L2_1', 'M_L_1', 5),
        ('M_L2_2', 'M_L_2', 5),
        ('M_L2_3', 'M_L_3', 5),
        ('M_L2_4', 'M_L_4', 5),

        # Diagonal connections
        ('M_L_0', 'M_L2_1', 5),
        ('M_L_1', 'M_L2_0', 5),
        ('M_L_1', 'M_L2_2', 5),
        ('M_L_2', 'M_L2_1', 5),
        ('M_L_2', 'M_L2_3', 5),
        ('M_L_3', 'M_L2_2', 5),
        ('M_L_3', 'M_L2_4', 5),
        ('M_L_4', 'M_L2_3', 5),

        # Backward diagonal connections
        ('M_L2_0', 'M_L_1', 5),
        ('M_L2_1', 'M_L_0', 5),
        ('M_L2_1', 'M_L_2', 5),
        ('M_L2_2', 'M_L_1', 5),
        ('M_L2_2', 'M_L_3', 5),
        ('M_L2_3', 'M_L_2', 5),
        ('M_L2_3', 'M_L_4', 5),
        ('M_L2_4', 'M_L_3', 5),
        
        # Vertical connections in additional left structure 1
        ('M_L2_0', 'M_L2_1', 5),
        ('M_L2_1', 'M_L2_2', 5),
        ('M_L2_2', 'M_L2_3', 5),
        ('M_L2_3', 'M_L2_4', 5),

        # Downward connections in additional left structure 1
        ('M_L2_1', 'M_L2_0', 5),
        ('M_L2_2', 'M_L2_1', 5),
        ('M_L2_3', 'M_L2_2', 5),
        ('M_L2_4', 'M_L2_3', 5),
        
        # Additional left structure 1 to additional left structure 2
        ('M_L2_0', 'M_L3_0', 5),
        ('M_L2_1', 'M_L3_1', 5),
        ('M_L2_2', 'M_L3_2', 5),
        ('M_L2_3', 'M_L3_3', 5),
        ('M_L2_4', 'M_L3_4', 5),

        # Additional left structure 2 to additional left structure 1
        ('M_L3_0', 'M_L2_0', 5),
        ('M_L3_1', 'M_L2_1', 5),
        ('M_L3_2', 'M_L2_2', 5),
        ('M_L3_3', 'M_L2_3', 5),
        ('M_L3_4', 'M_L2_4', 5),
        
        # Diagonal connections
        ('M_L2_0', 'M_L3_1', 5),
        ('M_L2_1', 'M_L3_0', 5),
        ('M_L2_1', 'M_L3_2', 5),
        ('M_L2_2', 'M_L3_1', 5),
        ('M_L2_2', 'M_L3_3', 5),
        ('M_L2_3', 'M_L3_2', 5),
        ('M_L2_3', 'M_L3_4', 5),
        ('M_L2_4', 'M_L3_3', 5),

        # Backward diagonal connections
        ('M_L3_0', 'M_L2_1', 5),
        ('M_L3_1', 'M_L2_0', 5),
        ('M_L3_1', 'M_L2_2', 5),
        ('M_L3_2', 'M_L2_1', 5),
        ('M_L3_2', 'M_L2_3', 5),
        ('M_L3_3', 'M_L2_2', 5),
        ('M_L3_3', 'M_L2_4', 5),
        ('M_L3_4', 'M_L2_3', 5),
        
        # Vertical connections in additional left structure 2
        ('M_L3_0', 'M_L3_1', 5),
        ('M_L3_1', 'M_L3_2', 5),
        ('M_L3_2', 'M_L3_3', 5),
        ('M_L3_3', 'M_L3_4', 5),

        # Downward connections in additional left structure 2
        ('M_L3_1', 'M_L3_0', 5),
        ('M_L3_2', 'M_L3_1', 5),
        ('M_L3_3', 'M_L3_2', 5),
        ('M_L3_4', 'M_L3_3', 5),
        
        # === BRIDGE CONNECTIONS (EXPANDED FOR ALL ROWS) ===
        # Additional left structure 2 to bridge column 1 (horizontal)
        ('M_L3_0', 'B_0_0', 5),
        ('M_L3_2', 'B_0_2', 5),
        ('M_L3_4', 'B_0_4', 5),

        # Bridge column 1 to additional left structure 2
        ('B_0_0', 'M_L3_0', 5),
        ('B_0_2', 'M_L3_2', 5),
        ('B_0_4', 'M_L3_4', 5),
        
        # Diagonal connections to bridge column 1
        ('M_L3_1', 'B_0_0', 5),
        ('M_L3_1', 'B_0_2', 5),
        ('M_L3_3', 'B_0_2', 5),
        ('M_L3_3', 'B_0_4', 5),

        # Backward diagonal connections to bridge column 1
        ('B_0_0', 'M_L3_1', 5),
        ('B_0_2', 'M_L3_1', 5),
        ('B_0_2', 'M_L3_3', 5),
        ('B_0_4', 'M_L3_3', 5),
        
        # Vertical connections in bridge column 1

        # Downward connections in bridge column 1
        
        # Bridge column 1 to bridge column 2 (horizontal)

        # Bridge column 2 to bridge column 1
        
        # Diagonal connections between bridge columns
        ('B_0_0', 'B_1_1', 5),
        ('B_0_2', 'B_1_1', 5),
        ('B_0_2', 'B_1_3', 5),
        ('B_0_4', 'B_1_3', 5),
        
        # Backward diagonal connections between bridge columns
        ('B_1_1', 'B_0_0', 5),
        ('B_1_1', 'B_0_2', 5),
        ('B_1_3', 'B_0_2', 5),
        ('B_1_3', 'B_0_4', 5),

        # Vertical connections in bridge column 2

        # Downward connections in bridge column 2
        
        # Bridge column 2 to additional right structure 1 (horizontal)
        ('B_1_1', 'M_R2_1', 5),
        ('B_1_3', 'M_R2_3', 5),

        # Additional right structure 1 to bridge column 2
        ('M_R2_1', 'B_1_1', 5),
        ('M_R2_3', 'B_1_3', 5),
        
        # Diagonal connections to additional right structure 1
        ('B_1_1', 'M_R2_0', 5),
        ('B_1_1', 'M_R2_2', 5),
        ('B_1_3', 'M_R2_2', 5),
        ('B_1_3', 'M_R2_4', 5),

        # Backward diagonal connections to additional right structure 1
        ('M_R2_0', 'B_1_1', 5),
        ('M_R2_2', 'B_1_1', 5),
        ('M_R2_2', 'B_1_3', 5),
        ('M_R2_4', 'B_1_3', 5),
        
        # === RIGHT STRUCTURE CONNECTIONS (SAME AS CONFIG_11) ===
        # Vertical connections in additional right structure 1
        ('M_R2_0', 'M_R2_1', 5),
        ('M_R2_1', 'M_R2_2', 5),
        ('M_R2_2', 'M_R2_3', 5),
        ('M_R2_3', 'M_R2_4', 5),

        # Downward connections in additional right structure 1
        ('M_R2_1', 'M_R2_0', 5),
        ('M_R2_2', 'M_R2_1', 5),
        ('M_R2_3', 'M_R2_2', 5),
        ('M_R2_4', 'M_R2_3', 5),
        
        # Additional right structure 1 to additional right structure 2
        ('M_R2_0', 'M_R3_0', 5),
        ('M_R2_1', 'M_R3_1', 5),
        ('M_R2_2', 'M_R3_2', 5),
        ('M_R2_3', 'M_R3_3', 5),
        ('M_R2_4', 'M_R3_4', 5),

        # Additional right structure 2 to additional right structure 1
        ('M_R3_0', 'M_R2_0', 5),
        ('M_R3_1', 'M_R2_1', 5),
        ('M_R3_2', 'M_R2_2', 5),
        ('M_R3_3', 'M_R2_3', 5),
        ('M_R3_4', 'M_R2_4', 5),
        
        # Diagonal connections
        ('M_R2_0', 'M_R3_1', 5),
        ('M_R2_1', 'M_R3_0', 5),
        ('M_R2_1', 'M_R3_2', 5),
        ('M_R2_2', 'M_R3_1', 5),
        ('M_R2_2', 'M_R3_3', 5),
        ('M_R2_3', 'M_R3_2', 5),
        ('M_R2_3', 'M_R3_4', 5),
        ('M_R2_4', 'M_R3_3', 5),

        # Backward diagonal connections
        ('M_R3_0', 'M_R2_1', 5),
        ('M_R3_1', 'M_R2_0', 5),
        ('M_R3_1', 'M_R2_2', 5),
        ('M_R3_2', 'M_R2_1', 5),
        ('M_R3_2', 'M_R2_3', 5),
        ('M_R3_3', 'M_R2_2', 5),
        ('M_R3_3', 'M_R2_4', 5),
        ('M_R3_4', 'M_R2_3', 5),
        
        # Vertical connections in additional right structure 2
        ('M_R3_0', 'M_R3_1', 5),
        ('M_R3_1', 'M_R3_2', 5),
        ('M_R3_2', 'M_R3_3', 5),
        ('M_R3_3', 'M_R3_4', 5),

        # Downward connections in additional right structure 2
        ('M_R3_1', 'M_R3_0', 5),
        ('M_R3_2', 'M_R3_1', 5),
        ('M_R3_3', 'M_R3_2', 5),
        ('M_R3_4', 'M_R3_3', 5),
        
        # Additional right structure 2 to right structure
        ('M_R3_0', 'M_R_0', 5),
        ('M_R3_1', 'M_R_1', 5),
        ('M_R3_2', 'M_R_2', 5),
        ('M_R3_3', 'M_R_3', 5),
        ('M_R3_4', 'M_R_4', 5),

        # Right structure to additional right structure 2
        ('M_R_0', 'M_R3_0', 5),
        ('M_R_1', 'M_R3_1', 5),
        ('M_R_2', 'M_R3_2', 5),
        ('M_R_3', 'M_R3_3', 5),
        ('M_R_4', 'M_R3_4', 5),
        
        # Diagonal connections
        ('M_R3_0', 'M_R_1', 5),
        ('M_R3_1', 'M_R_0', 5),
        ('M_R3_1', 'M_R_2', 5),
        ('M_R3_2', 'M_R_1', 5),
        ('M_R3_2', 'M_R_3', 5),
        ('M_R3_3', 'M_R_2', 5),
        ('M_R3_3', 'M_R_4', 5),
        ('M_R3_4', 'M_R_3', 5),

        # Backward diagonal connections
        ('M_R_0', 'M_R3_1', 5),
        ('M_R_1', 'M_R3_0', 5),
        ('M_R_1', 'M_R3_2', 5),
        ('M_R_2', 'M_R3_1', 5),
        ('M_R_2', 'M_R3_3', 5),
        ('M_R_3', 'M_R3_2', 5),
        ('M_R_3', 'M_R3_4', 5),
        ('M_R_4', 'M_R3_3', 5),
        
        # Vertical connections in right structure
        ('M_R_0', 'M_R_1', 5),
        ('M_R_1', 'M_R_2', 5),
        ('M_R_2', 'M_R_3', 5),
        ('M_R_3', 'M_R_4', 5),

        # Downward connections in right structure
        ('M_R_1', 'M_R_0', 5),
        ('M_R_2', 'M_R_1', 5),
        ('M_R_3', 'M_R_2', 5),
        ('M_R_4', 'M_R_3', 5),
        
        # Right structure to target nodes
        ('M_R_0', 'T_0', 5),
        ('M_R_1', 'T_1', 5),
        ('M_R_2', 'T_2', 5),
        ('M_R_3', 'T_3', 5),
        ('M_R_4', 'T_4', 5),

        # Target nodes to right structure
        ('T_0', 'M_R_0', 5),
        ('T_1', 'M_R_1', 5),
        ('T_2', 'M_R_2', 5),
        ('T_3', 'M_R_3', 5),
        ('T_4', 'M_R_4', 5),
        
        # Diagonal connections
        ('M_R_0', 'T_1', 5),
        ('M_R_1', 'T_0', 5),
        ('M_R_1', 'T_2', 5),
        ('M_R_2', 'T_1', 5),
        ('M_R_2', 'T_3', 5),
        ('M_R_3', 'T_2', 5),
        ('M_R_3', 'T_4', 5),
        ('M_R_4', 'T_3', 5),

        # Backward diagonal connections
        ('T_0', 'M_R_1', 5),
        ('T_1', 'M_R_0', 5),
        ('T_1', 'M_R_2', 5),
        ('T_2', 'M_R_1', 5),
        ('T_2', 'M_R_3', 5),
        ('T_3', 'M_R_2', 5),
        ('T_3', 'M_R_4', 5),
        ('T_4', 'M_R_3', 5),
        
        # Vertical connections between target nodes
        ('T_0', 'T_1', 5),
        ('T_1', 'T_2', 5),
        ('T_2', 'T_3', 5),
        ('T_3', 'T_4', 5),

        # Downward connections between target nodes
        ('T_1', 'T_0', 5),
        ('T_2', 'T_1', 5),
        ('T_3', 'T_2', 5),
        ('T_4', 'T_3', 5),
    ],
    'source': 'Super_S',
    'sink': 'Super_T',
    'positions': {
        # Super nodes
        'Super_S': (-1, 3),
        'Super_T': (10, 3),
        
        # Source nodes
        'S_0': (0, 0),
        'S_1': (0, 1),
        'S_2': (0, 2),
        'S_3': (0, 3),
        'S_4': (0, 4),
        
        # Left structure nodes
        'M_L_0': (1, 0),
        'M_L_1': (1, 1),
        'M_L_2': (1, 2),
        'M_L_3': (1, 3),
        'M_L_4': (1, 4),
        
        # Additional left structure nodes 1
        'M_L2_0': (2, 0),
        'M_L2_1': (2, 1),
        'M_L2_2': (2, 2),
        'M_L2_3': (2, 3),
        'M_L2_4': (2, 4),
        
        # Additional left structure nodes 2
        'M_L3_0': (3, 0),
        'M_L3_1': (3, 1),
        'M_L3_2': (3, 2),
        'M_L3_3': (3, 3),
        'M_L3_4': (3, 4),
        
        # Bridge nodes in column 4 (all rows)
        'B_0_0': (4, 0),
        'B_0_2': (4, 2),
        'B_0_4': (4, 4),
        
        # Bridge nodes in column 5 (all rows)
        'B_1_1': (5, 1),
        'B_1_3': (5, 3),
        
        # Additional right structure nodes 1
        'M_R2_0': (6, 0),
        'M_R2_1': (6, 1),
        'M_R2_2': (6, 2),
        'M_R2_3': (6, 3),
        'M_R2_4': (6, 4),
        
        # Additional right structure nodes 2
        'M_R3_0': (7, 0),
        'M_R3_1': (7, 1),
        'M_R3_2': (7, 2),
        'M_R3_3': (7, 3),
        'M_R3_4': (7, 4),
        
        # Right structure nodes
        'M_R_0': (8, 0),
        'M_R_1': (8, 1),
        'M_R_2': (8, 2),
        'M_R_3': (8, 3),
        'M_R_4': (8, 4),
        
        # Target nodes
        'T_0': (9, 0),
        'T_1': (9, 1),
        'T_2': (9, 2),
        'T_3': (9, 3),
        'T_4': (9, 4),
    }
}


# Update ALL_CONFIGS list to include CONFIG_12
ALL_CONFIGS = [CONFIG_1, CONFIG_2, CONFIG_3, CONFIG_4, CONFIG_5, CONFIG_6, CONFIG_7, CONFIG_8, CONFIG_9, CONFIG_10, CONFIG_11, CONFIG_12, CONFIG_13]

def get_config(config_name):
    """Get configuration by name"""
    for config in ALL_CONFIGS:
        if config['name'] == config_name:
            return config
    return None

def list_configs():
    """List all available configuration names"""
    return [config['name'] for config in ALL_CONFIGS]
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

# All configurations list
ALL_CONFIGS = [CONFIG_1, CONFIG_2, CONFIG_3, CONFIG_4]

def get_config(config_name):
    """Get configuration by name"""
    for config in ALL_CONFIGS:
        if config['name'] == config_name:
            return config
    return None

def list_configs():
    """List all available configuration names"""
    return [config['name'] for config in ALL_CONFIGS]
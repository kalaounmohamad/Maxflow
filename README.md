# Max-Flow Algorithm Implementations and Analysis

This project contains Python scripts to calculate and visualize maximum flow in a network.

## Core Analysis Scripts

### `Manual/edmonds_karp_networkx_imp.py`

A minimal script demonstrating `networkx` max-flow calculation on a hardcoded graph.

- **Purpose**: A simple, self-contained example of using the `networkx` library's `edmonds_karp` function.
- **How to Run**:
  ```sh
  python Manual/edmonds_karp_networkx_imp.py
  ```

### `edmonds_karp_networkx_func.py`

Analyzes graph configurations from `graph_configs.py` using the `networkx` library. It calculates the max flow and generates visualizations.

- **Key Functions**:
  - `analyze_flow_graph()`: Runs the full analysis for a graph.
  - `calculate_max_flow()`: Computes max flow using `networkx`.
  - `visualize_flow()`: Plots the network, showing flow vs. capacity, and saves images to categorized subfolders in `images_2d/`.
- **How to Run**:
  1.  Open `edmonds_karp_networkx_func.py`.
  2.  In the `if __name__ == "__main__":` block, uncomment the `analyze_flow_graph()` call for the desired configuration.
  3.  Run the script:
      ```sh
      python edmonds_karp_networkx_func.py
      ```

### `Manual/edmonds_karp_manual_imp.py`

This is the main, from-scratch implementation of the Edmonds-Karp algorithm. It features a modified version of the standard algorithm for finding augmenting paths.

- **Key Functions**:

  - `edmonds_karp()`: The main algorithm loop.
  - `bfs()`: Finds the shortest augmenting path using a standard Breadth-First Search.
  - `bfs_pq_lexico_sum()`: An alternative path-finding method that uses a priority queue. It selects paths based on a lexicographical cost to break ties:
    1.  It first minimizes the number of hops (shortest path).
    2.  Among paths with the same number of hops, it maximizes the flow capacity.

- **How to Run**:
  1.  Open `Manual/edmonds_karp_manual_imp.py`.
  2.  Set the `config_choice` variable to the index of the graph you want to analyze.
  3.  Run the script:
      ```sh
      python Manual/edmonds_karp_manual_imp.py
      ```

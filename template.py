import heapq
from collections import deque, defaultdict

class Graph:
    def __init__(self):
        # Each graph has a set of nodes and edges
        self.nodes = set()
        self.edges = defaultdict(list)  # adjacency list
        self.capacity = {}              # used for max flow

    def add_edge(self, u, v, weight, cap):
        # Add both directions since sidewalks are undirected
        self.nodes.update([u, v])
        self.edges[u].append((v, weight, cap))
        self.edges[v].append((u, weight, cap))

        # For max flow, store capacity for each direction
        self.capacity[(u, v)] = cap
        self.capacity[(v, u)] = 0  # reverse edge has 0 capacity

    def find_path_for_group(self, start, end, group_size):
        # Dijkstra-like search to find a path where all edges
        # can handle the given group size (used for safety)
        queue = [(0, start, [])]  # (cost, current_node, path_so_far)
        visited = set()

        while queue:
            cost, node, path = heapq.heappop(queue)
            if node in visited:
                continue
            path = path + [node]
            if node == end:
                return cost, path
            visited.add(node)

            for neighbor, weight, capacity in self.edges.get(node, []):
                # Only consider paths that can support the group size
                if neighbor not in visited and group_size <= capacity:
                    heapq.heappush(queue, (cost + weight, neighbor, path))

        return float("inf"), []  # no valid path found

    def bfs(self, source, sink, parent):
        # Breadth-First Search for augmenting path in residual graph
        visited = {node: False for node in self.nodes}
        queue = deque([source])
        visited[source] = True

        while queue:
            u = queue.popleft()
            for v in self.get_neighbors(u):
                # Only visit nodes reachable through non-full edges
                if not visited[v] and self.capacity[(u, v)] > 0:
                    parent[v] = u
                    visited[v] = True
                    if v == sink:
                        return True
                    queue.append(v)
        return False

    def get_neighbors(self, u):
        # Helper to extract just the neighbor names
        return [v for v, _, _ in self.edges[u]]

    def max_flow_with_paths(self, source, sink):
        # Edmonds-Karp algorithm with path tracking
        parent = {}
        flow = 0
        paths = []

        while self.bfs(source, sink, parent):
            # Find the bottleneck capacity along the path
            path_flow = float('inf')
            s = sink
            path = []
            while s != source:
                path_flow = min(path_flow, self.capacity[(parent[s], s)])
                path.append(s)
                s = parent[s]
            path.append(source)
            path.reverse()

            # Update the residual capacities along the path
            v = sink
            while v != source:
                u = parent[v]
                self.capacity[(u, v)] -= path_flow
                self.capacity[(v, u)] += path_flow
                v = u

            flow += path_flow
            paths.append((path, path_flow))

        return flow, paths

# -------------------------
# Initialize the SMC graph
# -------------------------

smc = Graph()

# Add campus sidewalks:
# weight = distance (meters), cap = max group size allowed on path
smc.add_edge("Library", "Galileo Hall", 100, 25)
smc.add_edge("Galileo Hall", "De La Salle Hall", 80, 20)
smc.add_edge("Library", "Soda Center", 150, 30)
smc.add_edge("Soda Center", "Dryden Hall", 60, 15)
smc.add_edge("Dryden Hall", "Dante Quad", 50, 10)
smc.add_edge("Dante Quad", "Chapel", 100, 15)
smc.add_edge("Chapel", "Filippi Academic Hall", 70, 20)
smc.add_edge("Library", "Filippi Academic Hall", 200, 30)

# -------------------------
# Example 1: Shortest Safe Path for a Group
# -------------------------

group_size = 18
start = "Library"
end = "Chapel"

# Find shortest distance path that supports group_size
distance, path = smc.find_path_for_group(start, end, group_size)

if path:
    print(f"Shortest safe path for group size {group_size}:")
    print(f"  Path: {' -> '.join(path)}")
    print(f"  Total distance: {distance} meters\n")
else:
    print(f"No safe path found for a group of size {group_size} from {start} to {end}\n")

# -------------------------
# Example 2: Max Flow and Paths Used
# -------------------------

# Create new graph for flow calculation only
# We ignore distance and only care about capacity
flow_graph = Graph()
flow_graph.add_edge("Library", "Galileo Hall", 0, 25)
flow_graph.add_edge("Galileo Hall", "De La Salle Hall", 0, 20)
flow_graph.add_edge("Library", "Soda Center", 0, 30)
flow_graph.add_edge("Soda Center", "Dryden Hall", 0, 15)
flow_graph.add_edge("Dryden Hall", "Dante Quad", 0, 10)
flow_graph.add_edge("Dante Quad", "Chapel", 0, 15)
flow_graph.add_edge("Chapel", "Filippi Academic Hall", 0, 20)
flow_graph.add_edge("Library", "Filippi Academic Hall", 0, 30)

# Compute max flow and get the paths that contributed to it
maxflow, flow_paths = flow_graph.max_flow_with_paths("Library", "Chapel")

print(f"Maximum group flow from Library to Chapel: {maxflow}")
print("Paths used and their flow:")
for p, f in flow_paths:
    print(f"  Path: {' -> '.join(p)} | Flow: {f}")

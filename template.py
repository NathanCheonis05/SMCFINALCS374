import heapq

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = {}

    def add_edge(self, u, v, weight, capacity):
        self.nodes.update([u, v])
        self.edges.setdefault(u, []).append((v, weight, capacity))
        self.edges.setdefault(v, []).append((u, weight, capacity))  # undirected

    def find_path_for_group(self, start, end, group_size):
        queue = [(0, start, [])]
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
                if neighbor not in visited and group_size <= capacity:
                    heapq.heappush(queue, (cost + weight, neighbor, path))
        return float("inf"), []

# Initialize the campus graph
smc = Graph()

# Add sidewalks (weight in meters, capacity is group limit estimate)
smc.add_edge("Library", "Galileo Hall", 100, 25)
smc.add_edge("Galileo Hall", "De La Salle Hall", 80, 20)
smc.add_edge("Library", "Soda Center", 150, 30)
smc.add_edge("Soda Center", "Dryden Hall", 60, 15)
smc.add_edge("Dryden Hall", "Dante Quad", 50, 10)
smc.add_edge("Dante Quad", "Chapel", 100, 15)
smc.add_edge("Chapel", "Filippi Academic Hall", 70, 20)
smc.add_edge("Library", "Filippi Academic Hall", 200, 30)

# Example path search
group_size = 18
start = "Library"
end = "Chapel"

distance, path = smc.find_path_for_group(start, end, group_size)

if path:
    print(f"Group size: {group_size}")
    print(f"Path from {start} to {end}: {path} with total distance {distance} meters")
else:
    print(f"No safe path found for a group of size {group_size} from {start} to {end}")

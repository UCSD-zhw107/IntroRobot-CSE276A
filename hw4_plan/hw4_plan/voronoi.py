import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, LineString
import heapq




class voronoi_planner():
    def __init__(self, obstacle, boundary,start, goal,radius = 0.09):
        self.obstacle = self.interpolate_edge(obstacle)
        self.boundary = self.interpolate_edge(boundary)
        self.radius = radius
        self.vertices = None
        self.edges = None
        self.start = start
        self.goal = goal

    def interpolate_edge(self,coords, num_points=10):

        filled_coords = []
        num_coords = len(coords)
        for i in range(num_coords):
            start, end = coords[i], coords[(i + 1) % num_coords]
            x_values = np.linspace(start[0], end[0], num_points)
            y_values = np.linspace(start[1], end[1], num_points)
            filled_coords.extend(zip(x_values, y_values))
        return np.array(filled_coords)

    def mapCSpace(self, points, radius):
        polygon = Polygon(points).buffer(radius)
        ploy_coord = self.interpolate_edge(np.array(polygon.exterior.coords))
        return ploy_coord

    def runVoronoi(self, points):
        return Voronoi(points)

    def filter_voronoi_with_obstacle(self,vor, obstacle_coords):
        obstacle = Polygon(obstacle_coords)
        valid_vertices = []
        vertices_map = {}

        for i, vertex in enumerate(vor.vertices):
            point = Point(vertex)
            if not obstacle.contains(point):
                vertices_map[i] = len(valid_vertices)
                valid_vertices.append(vertex)

        valid_edges = []
        for ridge_vertices in vor.ridge_vertices:
            if -1 in ridge_vertices:
                continue
            p1 = vor.vertices[ridge_vertices[0]]
            p2 = vor.vertices[ridge_vertices[1]]
            line = LineString([p1, p2])

            if not obstacle.contains(line) and not line.intersects(obstacle):
                if ridge_vertices[0] in vertices_map and ridge_vertices[1] in vertices_map:
                    new_edge = [vertices_map[ridge_vertices[0]],
                                vertices_map[ridge_vertices[1]]]
                    valid_edges.append(new_edge)
        return np.array(valid_vertices), np.array(valid_edges)

    def build_graph(self, vertices, edges):
        graph = {i: [] for i in range(len(vertices))}
        for edge in edges:
            p1, p2 = edge
            distance = np.linalg.norm(vertices[p1] - vertices[p2])
            graph[p1].append((p2, distance))
            graph[p2].append((p1, distance))
        return graph

    def astar_search(self,graph, vertices, start_idx, goal_idx):
        open_set = [(0, start_idx, [])]
        heapq.heapify(open_set)

        g_score = {i: float('inf') for i in graph}
        g_score[start_idx] = 0

        while open_set:
            _, current, path = heapq.heappop(open_set)
            path = path + [current]
            if current == goal_idx:
                return path

            for neighbor, distance in graph[current]:
                tentative_g_score = g_score[current] + distance

                if tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + np.linalg.norm(vertices[neighbor] - vertices[goal_idx])
                    heapq.heappush(open_set, (f_score, neighbor, path))
        return None

    def rdp_simplify_path(self, path, epsilon=0.07):
        if len(path) < 3:
            return path
        start, end = path[0], path[-1]
        line = np.array(end) - np.array(start)
        line_len = np.linalg.norm(line)
        line_dir = line / line_len if line_len > 0 else line
        distances = np.cross(line_dir, np.array(path) - start)
        distances = np.abs(distances)

        max_dist_idx = np.argmax(distances)
        max_dist = distances[max_dist_idx]

        if max_dist > epsilon:
            left_path = self.rdp_simplify_path(path[:max_dist_idx + 1], epsilon)
            right_path = self.rdp_simplify_path(path[max_dist_idx:], epsilon)
            return np.vstack((left_path[:-1], right_path))
        else:
            return np.array([start, end])

    def plot_voronoi(self, voronoi, path=None, obstacle_coords=None, boundary_coords=None):

        vertices = voronoi['vertices']
        edges = voronoi['edges']
        plt.figure(figsize=(8, 8))


        for edge in edges:
            p1, p2 = vertices[edge[0]], vertices[edge[1]]
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', linestyle='-',
                     label='Filtered Edges' if edge[0] == edges[0][0] else None)


        if obstacle_coords is not None:
            plt.plot(obstacle_coords[:, 0], obstacle_coords[:, 1], color='red', label='Obstacle', linewidth=2)


        if boundary_coords is not None:
            plt.plot(boundary_coords[:, 0], boundary_coords[:, 1], color='green', label='Boundary', linewidth=2)

        if path is not None:
            start = path['start']
            goal = path['goal']
            path_coords = path['path']
            plt.plot(path_coords[:, 0], path_coords[:, 1], color='red', label='Path', linewidth=2)
            plt.scatter([start[0]], [start[1]], color='blue', label='Start')
            plt.scatter([goal[0]], [goal[1]], color='blue', label='Goal')


        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Filtered Voronoi Diagram')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

    def calculate_path_orientations(self, waypoints):
        orientations = []
        num_waypoints = len(waypoints)
        orientations.append(0)
        for i in range(1, num_waypoints):
            prev = waypoints[i - 1]
            current = waypoints[i]
            dx = current[0] - prev[0]
            dy = current[1] - prev[1]
            orientation = np.arctan2(dy, dx)
            orientation = (orientation + np.pi) % (2 * np.pi) - np.pi
            orientations.append(orientation)
        return np.array(orientations)

    def plan_path(self):
        obstacle_ploy = self.mapCSpace(self.obstacle, self.radius)
        boundary_ploy = self.mapCSpace(self.boundary, -self.radius)
        points = np.vstack((obstacle_ploy, boundary_ploy))
        vor = self.runVoronoi(points)
        self.vertices, self.edges = self.filter_voronoi_with_obstacle(vor, obstacle_ploy)
        graph = self.build_graph(self.vertices, self.edges)
        start_idx = np.argmin(np.linalg.norm(self.vertices - self.start, axis=1))
        goal_idx = np.argmin(np.linalg.norm(self.vertices - self.goal, axis=1))
        path_indices = self.astar_search(graph, self.vertices, start_idx, goal_idx)
        path_coords = self.vertices[path_indices]
        if path_coords is not None:
            path_coords = np.vstack([self.start, path_coords])
            path_coords = np.vstack([path_coords, self.goal])
            path_coords = self.rdp_simplify_path(path_coords)
        orientation  = self.calculate_path_orientations(path_coords)
        return np.hstack((path_coords, orientation.reshape(-1, 1)))




def main():
    start_point = [0, 0]
    goal_point = [0.9, 1.37]
    obstacle = np.array([[0.19, 0.67],
                         [0.53, 0.67],
                         [0.53, 0.93],
                         [0.19, 0.93]])
    boundary = np.array([
        [1.7, -0.74],
        [-0.9, -0.74],
        [-0.9, 1.7],
        [1.7, 1.7]
    ])
    v = voronoi_planner(obstacle, boundary,start_point, goal_point)
    waypoints = v.plan_path()
    print(waypoints)



if __name__ == "__main__":
    main()
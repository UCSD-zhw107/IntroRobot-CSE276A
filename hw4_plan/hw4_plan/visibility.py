import heapq
import numpy as np
from shapely.geometry import Polygon,LineString,box
from itertools import combinations

class visibility_planner():
    def __init__(self,obstacle, boundary,start, goal,radius = 0.09):
        self.obstacle = self.interpolate_edge(obstacle)
        self.boundary = self.interpolate_edge(boundary)
        self.radius = radius
        self.vertices = None
        self.edges = None
        self.start = start
        self.goal = goal

    def mapCSpace(self, points, radius):
        polygon = Polygon(points).buffer(radius)
        ploy_coord = self.interpolate_edge(np.array(polygon.exterior.coords))
        return ploy_coord

    def interpolate_edge(self, coords, num_points=10):
        filled_coords = []
        num_coords = len(coords)
        for i in range(num_coords):
            start, end = coords[i], coords[(i + 1) % num_coords]
            x_values = np.linspace(start[0], end[0], num_points)
            y_values = np.linspace(start[1], end[1], num_points)
            filled_coords.extend(zip(x_values, y_values))
        return np.array(filled_coords)

    def is_visible(self,i, j, points, obstacle):
        point1 = points[i]
        point2 = points[j]
        edge = LineString([point1, point2])
        return not edge.intersects(obstacle)

    def run_visibility(self, points, obstacle_poly):
        visibility_graph = []
        for i, j in combinations(range(len(points)), 2):
            if self.is_visible(i, j, points, obstacle_poly):
                visibility_graph.append((i, j))
        return visibility_graph

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

    def calculate_path_orientations(self,waypoints):
    
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
        minx, miny, maxx, maxy = Polygon(obstacle_ploy).bounds
        bounding_box = box(minx, miny, maxx, maxy)
        square_vertices = np.array(bounding_box.exterior.coords)[:-1]
        points = np.vstack((self.start, self.goal))
        points = np.vstack((points, square_vertices))
        visibility_graph = self.run_visibility(points, Polygon(obstacle_ploy))
        graph = self.build_graph(points, visibility_graph)
        start_idx = np.argmin(np.linalg.norm(points - self.start, axis=1))
        goal_idx = np.argmin(np.linalg.norm(points - self.goal, axis=1))
        path_indices = self.astar_search(graph, points, start_idx, goal_idx)
        path_coords = points[path_indices]
        orientation = self.calculate_path_orientations(path_coords)
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
    v = visibility_planner(obstacle, boundary,start_point, goal_point, radius=0.13)
    waypoints = v.plan_path()
    print(waypoints)

if __name__ == "__main__":
    main()
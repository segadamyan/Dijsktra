import sys

import numpy as np


class Graph:
    DEFAULT_SIZE = 5

    def __init__(self, adjacency: list):
        if adjacency is None:
            self.__adjacency: list = [[0 for _ in range(Graph.DEFAULT_SIZE)]
                                      for _ in range(Graph.DEFAULT_SIZE)]
        else:
            self.__validate(adjacency)
            self.__adjacency: list = adjacency
        self.__vertices: int = len(adjacency)

    def dijkstra(self, src: int, view: bool = False) -> dict:
        # Dijkstra's single source the shortest path algorithm for a graph(adjacency matrix)

        distance: dict = {i: sys.maxsize for i in range(self.__vertices)}
        distance[src] = 0
        shortest_path: set = set()

        for _ in range(self.__vertices):
            # Pick the shortest vertex's index
            shortest_index = self.__min_distance(distance, shortest_path)

            # Put the minimum distance vertex in the shortest path tree
            shortest_path.add(shortest_index)

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for y in range(self.__vertices):
                if self.__adjacency[shortest_index][y] > 0 and y not in shortest_path and \
                        distance[y] > distance[shortest_index] + self.__adjacency[shortest_index][y]:
                    distance[y] = distance[shortest_index] + self.__adjacency[shortest_index][y]
        if view:
            self.__print_solution(distance)
        return distance

    @staticmethod
    def __validate(adjacency: list):
        assert len(adjacency) == len(adjacency[0]), "Matrix must be square."
        assert np.transpose(adjacency) == np.array(adjacency), "Matrix must be symmetric."

    def __print_solution(self, dist: dict) -> None:
        print("Vertex \tDistance from Source")
        for node in range(self.__vertices):
            print(node, " --- ", dist[node])

    def __min_distance(self, distance: dict, shortest_path: set) -> int:
        # A utility function to find the vertex with minimum distance value, from the set of vertices not yet included
        # in the shortest path tree

        # Initialize minimum distance for next node
        min_distance: int = sys.maxsize

        for v in range(self.__vertices):
            if distance[v] < min_distance and v not in shortest_path:
                min_distance = distance[v]
                min_index = v

        return min_index

from collections import defaultdict
from collections import namedtuple
INF = 999999999999
Coordinate = namedtuple("Coordinate", ["x", "y"])


class Queue:
    def __init__(self):
        self.list = []

    def push(self, item):
        self.list.insert(0, item)

    def pop(self):
        return self.list.pop()

    def is_empty(self):
        return len(self.list) == 0


class Graph:
    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.graph_matrix = defaultdict(dict)
        for y in range(height):
            for x in range(width):
                coordinate = Coordinate(x, y)
                self.graph_matrix[coordinate] = defaultdict(list)

        for coordinate_a in self.graph_matrix:
            for coordinate_b in self.graph_matrix:
                self.graph_matrix[coordinate_a][coordinate_b] = INF

        self.distance_matrix = defaultdict(dict)
        for y in range(height):
            for x in range(width):
                coordinate = Coordinate(x, y)
                self.distance_matrix[coordinate] = defaultdict(list)

        for coordinate_a in self.distance_matrix:
            for coordinate_b in self.distance_matrix:
                self.distance_matrix[coordinate_a][coordinate_b] = INF

        self.flag = defaultdict(list)
        for y in range(height):
            for x in range(width):
                coordinate = Coordinate(x, y)
                self.flag[coordinate] = 0

        self.number_of_vertices = (height * width) * (height * width)

    def print_distance_matrix(self):
        for coordinate_a in self.distance_matrix:
            for coordinate_b in self.distance_matrix:
                print(self.distance_matrix[coordinate_a][coordinate_b], end=" ")
            print()
        print()

    def print_graph_matrix(self):
        for coordinate_a in self.graph_matrix:
            for coordinate_b in self.graph_matrix:
                print(self.graph_matrix[coordinate_a][coordinate_b], end=" ")
            print()

    def add_edge(self, u, v, d):
        self.graph_matrix[u][v] = d
        self.graph_matrix[v][u] = d
        self.graph_matrix[u][u] = 0
        self.graph_matrix[v][v] = 0

    def all_pair_shortest_path(self):
        for y in range(0, self.height):
            for x in range(0, self.width):
                self.dijkstra(Coordinate(x, y))

    def dijkstra(self, source):
        queue = Queue()
        queue.push(source)
        while not queue.is_empty():
            current_vertex = queue.pop()
            if self.flag[current_vertex] == 1:
                for y in range(0, self.height):
                    for x in range(0, self.width):
                        vertex = Coordinate(x, y)
                        if self.distance_matrix[source][current_vertex] == INF:
                            sum_edge = self.distance_matrix[current_vertex][vertex]
                        else:
                            sum_edge = self.distance_matrix[source][current_vertex] \
                                       + self.distance_matrix[current_vertex][vertex]
                        if sum_edge < self.distance_matrix[source][vertex]:
                            self.distance_matrix[source][vertex] = sum_edge
            else:
                for neighbour in self.get_neighbors(current_vertex):
                    if self.distance_matrix[source][current_vertex] == INF:
                        sum_edge = self.graph_matrix[current_vertex][neighbour]
                    else:
                        sum_edge = self.distance_matrix[source][current_vertex] \
                                   + self.graph_matrix[current_vertex][neighbour]
                    if sum_edge < self.distance_matrix[source][neighbour]:
                        self.distance_matrix[source][neighbour] = sum_edge
                        queue.push(neighbour)
        self.flag[source] = 1

    def sum_distance_edges(self, a, b):
        if self.distance_matrix[a[0]][a[1]] == INF:
            return self.graph_matrix[b[0]][b[1]]
        else:
            return self.distance_matrix[a[0]][a[1]] + self.graph_matrix[b[0]][b[1]]

    def get_neighbors(self, u):
        neighbors = []
        for (distance, v) in enumerate(self.graph_matrix[u]):
            if distance != INF:
                neighbors.append(v)
        return neighbors


def main():
    graph = read_graph_from_file("input", "r")
    graph.all_pair_shortest_path()
    graph.print_graph_matrix()
    print("------------------------------")
    graph.print_distance_matrix()


def read_graph_from_file(file_name, mode):
    file = open(file_name, mode)
    n = [int(x) for x in next(file).split()]
    graph = Graph(n[0], n[0])
    for line in file:
        x, y, x1, y1, d = [int(it) for it in line.strip().split(' ')]
        graph.add_edge(Coordinate(x, y), Coordinate(x1, y1), d)
    file.close()
    return graph


if __name__ == '__main__':
    main()

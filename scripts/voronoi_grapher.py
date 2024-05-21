import cv2

from numpy.typing import NDArray
import scipy.ndimage as ndimage
import numpy as np

from sklearn.cluster import KMeans

class VoronoiGrapher:

    def __init__(self, map_image = None) -> None:
        self.CLUSTER_PERCENT = 0.1

        self.map_ = map_image
        self.process_map()

    def get_graph(self):
        """Gets graph processed from the input map"""
        return self.graph_

    def find_local_minima(self, distance_field):
        """
        Finds local minima in a distance field using iterative search.

        Args:
            distance_field (np.ndarray): Distance field image.

        Returns:
            local_minima: List of coordinates (tuples of (x, y)) representing local minima.
        """

        local_minima = []
        height, width = distance_field.shape

        for y in range(height):
            for x in range(width):
                # Check if current pixel is lower than its neighbors (excluding edges)
                if (x > 0 and distance_field[y, x] < distance_field[y, x-1]) or \
                    (x < width-1 and distance_field[y, x] < distance_field[y, x+1]) or \
                    (y > 0 and distance_field[y, x] < distance_field[y-1, x]) or \
                    (y < height-1 and distance_field[y, x] < distance_field[y+1, x]):
                        continue  # Not a minimum

                local_minima.append((x, y))

        return local_minima

    def create_graph(self, nodes, k=4):
        """
        Creates a graph with given array of nodes,

        Args:
        endpoints = find_local_minima(distance_field)
            k (int): number of closest connected nodes
        
        Returns:
            graph: An adjacency list representing the connected graph
        """

        num_nodes = nodes.shape[0]
        graph = [[] for _ in range(num_nodes)]  # Initialize empty adjacency list

        print('creating graph...')
        # Iterate through each node
        for i, node in enumerate(nodes):
            # Calculate distances from the current node to all other nodes
            distances = np.linalg.norm(nodes - node, axis=1)
            
            # Get indices of the k nearest neighbors (excluding the node itself)
            nearest_indices = np.argsort(distances)[1:k+1]

            print(f"Node {i} neighbors: {nearest_indices}")

            # Add edges to the adjacency list
            for neighbor_index in nearest_indices:
                graph[i].append(neighbor_index)

        return graph

    def process_map(self, map=None):
        if map is None:
           map = self.map_

        # pre-process the map image
        if isinstance(map, str):
            input_image = cv2.imread(map)
            print(f'received image of size, {np.shape(input_image)}')

            # convert to a grayscale image
            gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
            print(f'converted to gray of size, {np.shape(gray)}')
        
        # no need to pre-process if directly from map
        gray = map
        
        # convert to binary image
        _, bin = cv2.threshold(gray, 253, 255, cv2.THRESH_BINARY)

        # compute distance field
        distance_field = ndimage.distance_transform_edt(input=bin, sampling=6)

        endpoints = self.find_local_minima(distance_field)
        clipped_endpoints = [point for point in endpoints if distance_field[point[1], point[0]] > 0]

        # KMeans clustering
        num_clusters = round(len(clipped_endpoints) * self.CLUSTER_PERCENT)
        kmeans = KMeans(n_clusters=num_clusters)
        kmeans.fit(clipped_endpoints)
        centers = np.round(kmeans.cluster_centers_).astype(int)

        self.graph_ = self.create_graph(centers, k=4)

    
        # visualise connected nodes
        for index, node in enumerate(self.graph_):
            for connected in node:
                cv2.line(distance_field, centers[index], centers[connected], (255,0,0), 1)

        return distance_field

if __name__ == '__main__':
    v_graph = VoronoiGrapher('map.pgm')

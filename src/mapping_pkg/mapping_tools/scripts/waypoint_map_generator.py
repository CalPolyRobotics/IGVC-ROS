#!/usr/bin/env python
import os
import csv
import rospy
import numpy as np
import support_nav_utils.nav_utils as nav_utils
import support_nav_utils.collision_utils as collision_utils
import support_nav_utils.projection_utils as projection_utils


class WaypointMapGenerator:
    """
    Generate a waypoint map and write it to a file.
    """
    def __init__(self, waypoint_map_dir, waypoint_map_name):
        self.waypoint_map_dir = waypoint_map_dir
        self.waypoint_map_name = waypoint_map_name
        self.on_road_graph = []
        self.off_road_graphs = []
        self.map = []

        self.load_graphs()  # load recorded waypoint graphs

    def load_graphs(self):
        """
        Load on and off road recorded waypoints needed to generate waypoint map.
        """
        for waypoint_file in os.listdir(self.waypoint_map_dir):
            lats = []
            lons = []

            with open(self.waypoint_map_dir + '/' + waypoint_file) as csvfile:
                csvreader = csv.reader(csvfile, delimiter=',')

                for line, row in enumerate(csvreader):
                    if line != 0:
                        lats.append(float(row[0]))
                        lons.append(float(row[1]))

            x, y = projection_utils.wgs_to_utm(lats, lons)

            if 'on_road' in waypoint_file:
                self.on_road_graph = [list(coords) for coords in zip(x, y)]
            elif 'off_road' in waypoint_file:
                self.off_road_graphs.append([list(coords) for coords in zip(x, y)])

    def generate_map(self):
        """
        Generate a waypoint map with valid edges from the individual on and off
        road graphs. 
        """
        edge_matrix = self.find_valid_edges()  # compute adjacency matrix

        # create on road part of map from adjacency matrix
        for row in range(len(self.on_road_graph)):
            point = self.on_road_graph[row]
            edges = np.nonzero(edge_matrix[row,:])
            self.map.append([row, True, point[0], point[1], edges])

        start_id = len(self.on_road_graph)  # starting id of off road graphs

        # create off road part of map from off road graphs
        for off_road_graph in self.off_road_graphs:
            for idx in range(len(off_road_graph)):
                point_id = start_id + idx
                point = off_road_graph[idx]

                if idx == 0:  # first point in graph
                    edges = [start_id + len(off_road_graph) - 1, start_id + idx + 1]
                elif idx == len(off_road_graph) - 1:  # last point in graph
                    edges = [start_id + idx - 1, start_id] 
                else:
                    edges = [start_id + idx - 1, start_id + idx + 1]

                self.map.append([point_id, False, point[0], point[1], edges])

            start_id += len(off_road_graph)  # update start id for next graph

        self.write_map_to_file()  # write map to file

    def find_valid_edges(self):
        """
        Find valid edges in the on road map. This works by finding edges that
        aren't greater than length > MAX_DIST and don't intersect any off road
        regions.

        Returns:
            edge_matrix: a NxN adjacency matrix
        """
        edges = np.ones((len(self.on_road_graph), len(self.on_road_graph)))
        edges[np.diag_indices(len(self.on_road_graph))] = 0  # can't have an edge to yourself

    def write_map_to_file(self):
        """
        Write map to file to later be loaded.
        """
        with open(waypoint_map_name, 'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(['ID', 'OnRoad', 'X', 'Y', 'Edges'])

            for map_point in self.map:
                csvwriter.writerow(map_point)


if __name__ == '__main__':
    rospy.init_node('waypoint_map_generator')

    # load in params from the parameter server
    waypoint_map_dir = rospy.get_param('waypoint_map_dir')
    waypoint_map_name = rospy.get_param('waypoint_map_name')

    # generate waypoint map and write it to file
    waypoint_map_generator = WaypointMapGenerator(waypoint_map_dir, waypoint_map_name)
    waypoint_map_generator.generate_map()

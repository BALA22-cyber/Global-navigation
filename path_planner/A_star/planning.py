import copy
import warnings

import numpy as np
import heapq
from typing import List


class Node:
    def __init__(self, parent, position, cost_to_go, cost_to_come, idx):
        self.parent = parent
        self.position = position
        self.cost_to_go = cost_to_go
        self.cost_to_come = cost_to_come
        self.idx = idx

    @property
    def cost(self):
        return self.cost_to_come + self.cost_to_go

    def __lt__(self, other):
        if self.cost < other.cost:
            return True
        else:
            return False

    def __le__(self, other):
        if self.cost <= other.cost:
            return True
        else:
            return False


class PlanningMethods:
    def __init__(self, map_resolution, step_size, target_arriving_threshold, max_iteration,
                 path_collision_threshold):
        """
        interface: calculate_paths_separately, calculate_paths_together
        """
        self.map_resolution = map_resolution
        self.step_size = int(step_size / map_resolution)
        self.target_arriving_threshold = target_arriving_threshold / map_resolution
        self.max_iteration = max_iteration
        self.path_collision_threshold = int(path_collision_threshold / map_resolution)

        self.all_targets_paths = []
        self.open_nodes = []
        self.all_nodes = []
        self.all_positions = []
        self.idx = 0
        self.iteration = 0

    @staticmethod
    def bresenham_line(start_point, end_point):
        """
        Generate a list of indices on a 2D grid that lie on a line
        between two points using the Bresenham's line algorithm.
        """
        indices = []
        dx = abs(end_point[0] - start_point[0])
        dy = abs(end_point[1] - start_point[1])
        x, y = start_point[0], start_point[1]
        sx = -1 if start_point[0] > end_point[0] else 1
        sy = -1 if start_point[1] > end_point[1] else 1
        if dx > dy:
            err = dx / 2.0
            while x != end_point[0]:
                indices.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != end_point[1]:
                indices.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        indices.append((x, y))
        return np.array(indices, dtype=int)

    def _check_path_collision(self, path, global_map):
        for idx in range(len(path)):
            point = path[idx]
            points = []
            for x in range(-self.path_collision_threshold, 1 + self.path_collision_threshold, 1):
                for y in range(-self.path_collision_threshold, 1 + self.path_collision_threshold, 1):
                    if np.linalg.norm([x, y]) <= self.path_collision_threshold:
                        rx, ry = point[0] + x, point[1] + y
                        if global_map.shape[0] > rx >= 0 and global_map.shape[1] > ry >= 0:
                            points.append([rx, ry])
            points = np.asarray(points)
            if len(points) > 0 and np.sum(global_map[points[:, 0], points[:, 1]]):
                return True
        return False

    def _select_grid_action(self, global_map, current_position, path, start_position):
        path = np.array(path)
        children = []
        choices = np.array([[-1, 1], [0, 1], [1, 1], [-1, 0], [1, 0], [-1, -1], [0, -1], [1, -1]], dtype=int)
        for neighbor in choices:
            neighbor_position = current_position + neighbor
            # if visited before:
            all_xy = abs(path[:, 0] - neighbor_position[0]) + abs(path[:, 1] - neighbor_position[1])
            if np.any(all_xy == 0):
                continue
            # if in collision:
            total_path = np.asarray([current_position, neighbor_position], dtype=int) * self.step_size + start_position
            total_path = self.bresenham_line(start_point=total_path[0], end_point=total_path[1])
            if self._check_path_collision(path=total_path, global_map=global_map):
                continue
            children.append(neighbor_position)
        return children

    def _check_parent_node(self, open_nodes, child_position, parent_node: Node):
        cost_to_come = np.linalg.norm(child_position - parent_node.position) * self.step_size + parent_node.cost_to_come
        current_parent_node = parent_node
        for node in open_nodes:
            if abs(node.position[0] - child_position[0]) <= 1 and abs(node.position[1] - child_position[1]) <= 1:
                current_cost_to_come = node.cost_to_come + np.linalg.norm(
                    child_position - node.position) * self.step_size
                if current_cost_to_come < cost_to_come:
                    current_parent_node = node
                    cost_to_come = current_cost_to_come
        return current_parent_node.idx, cost_to_come

    def _trace_back(self, all_nodes: List[Node], current_node: Node, start_position):
        path = [current_node.position * self.step_size + start_position]
        while current_node.parent is not None or current_node.idx != 0:
            current_node = all_nodes[current_node.parent]
            path.append(current_node.position * self.step_size + start_position)
        return path

    def _check_straight_line(self, global_map, current_position, target):
        # Test straight line:
        path = self.bresenham_line(current_position, target)
        if not self._check_path_collision(path=path, global_map=global_map):
            return path
        else:
            return None

    def _test_nodes_indices(self, all_nodes: List[Node], all_poses):
        for i in range(len(all_poses)):
            assert np.any(all_nodes[i].position == all_poses[i]), "node {} position is not aligned".format(i)
            assert all_nodes[i].idx == i, "node {} index is not aligned".format(i)
            if all_nodes[i].parent is not None:
                assert abs(all_nodes[all_nodes[i].parent].position[0] - all_nodes[i].position[0]) <= 1 and \
                       abs(all_nodes[all_nodes[i].parent].position[1] - all_nodes[i].position[1]) <= 1, \
                    "parent index of node {} is not correct".format(i)

    def _straight_line_to_waypoints(self, start, target):
        gradience = (target[1] - start[1]) / (target[0] - start[0])
        distance = np.linalg.norm(target - start)
        number = int(distance / self.step_size)
        path = []
        step_size = np.asarray([self.step_size / (gradience + 1), self.step_size * gradience / (gradience + 1)])
        for i in range(number):
            path.append(start + step_size * i)
        return np.asarray(path, dtype=int)

    def _update_nodes(self, all_nodes: List[Node], open_nodes: List[Node], target_position, start_position):
        for node in all_nodes:
            node.cost_to_go = np.linalg.norm(node.position * self.step_size + start_position - target_position)

        new_open_nodes = []
        while len(open_nodes) > 0:
            node = heapq.heappop(open_nodes)
            node.cost_to_go = np.linalg.norm(node.position * self.step_size + start_position - target_position)
            heapq.heappush(new_open_nodes, node)
        return all_nodes, new_open_nodes

    def _check_path(self, paths):
        for path in paths:
            if len(path) == 1 and np.any(path[0] == np.inf):
                return True
        return False

    def _find_closest_node(self, all_nodes: List[Node], target_position, start_position, all_positions):
        distances = np.linalg.norm(np.asarray(all_positions) * self.step_size + start_position - target_position,
                                   axis=1)
        index = np.argmin(distances)
        return all_nodes[index]

    def _reset_parameters(self, target_positions):
        self.all_targets_paths = [[np.inf] for i in range(len(target_positions))]
        self.open_nodes = []
        self.all_nodes = []
        self.all_positions = []
        self.idx = 0
        self.iteration = 0

    def _get_single_path(self, global_map, start_position, target_positions, tidx):
        # print(tidx, target_position, idx)
        target_position = target_positions[tidx]
        while len(
                self.open_nodes) > 0 and self.iteration < self.max_iteration:  # and self._check_path(all_targets_paths):
            self.iteration += 1
            node = heapq.heappop(self.open_nodes)
            children = self._select_grid_action(global_map=global_map, current_position=node.position,
                                                path=self.all_positions, start_position=start_position)
            if len(children) == 0:
                continue
            for child_position in children:
                current_real_position = child_position * self.step_size + start_position
                parent_idx, cost_to_come = self._check_parent_node(self.open_nodes, child_position,
                                                                   parent_node=node)
                self.idx += 1
                current_cost_to_go = np.linalg.norm(current_real_position - target_position)
                child_node = Node(parent=parent_idx, position=child_position, idx=self.idx,
                                  cost_to_come=cost_to_come,
                                  cost_to_go=current_cost_to_go)
                self.all_positions.append(child_position)
                self.all_nodes.append(child_node)
                heapq.heappush(self.open_nodes, child_node)

                if current_cost_to_go < self.target_arriving_threshold:
                    final_path = self._trace_back(all_nodes=self.all_nodes, current_node=child_node,
                                                  start_position=start_position)
                    final_path.reverse()
                    final_path.append(target_position)
                    self.all_targets_paths[tidx] = copy.deepcopy(final_path)
                    return
            if self.iteration >= self.max_iteration:
                warnings.warn("stop by iterations")
            if len(self.open_nodes) <= 0:
                warnings.warn("stop by explored all the map")
        return

    def _A_star_all(self, start_position, target_positions, global_map):
        """
        multiple targets with the same start position, where the explored nodes would be updated with different heuristics
        """
        self._reset_parameters(target_positions=target_positions)
        for tidx in range(len(target_positions)):
            if np.sum(self.all_targets_paths[tidx]) != np.inf:
                continue
            target_position = target_positions[tidx]
            if len(self.open_nodes) != 0 or len(self.all_positions) != 0:
                self.all_nodes, self.open_nodes = self._update_nodes(all_nodes=self.all_nodes,
                                                                     open_nodes=self.open_nodes,
                                                                     target_position=target_position,
                                                                     start_position=start_position)
                self._test_nodes_indices(all_nodes=self.all_nodes, all_poses=self.all_positions)
                saving_node = self._find_closest_node(all_nodes=self.all_nodes, target_position=target_position,
                                                      start_position=start_position, all_positions=self.all_positions)
                if saving_node.cost_to_go < self.target_arriving_threshold:
                    new_path = self._trace_back(all_nodes=self.all_nodes, current_node=saving_node,
                                                start_position=start_position)
                    new_path.reverse()
                    new_path.append(target_position)
                    self.all_targets_paths[tidx] = copy.deepcopy(new_path)
                    continue
            else:
                start_node = Node(parent=None, position=np.array([0, 0], dtype=int), idx=self.idx,
                                  cost_to_go=np.linalg.norm(start_position - target_position), cost_to_come=0)
                # saving_node = None
                heapq.heappush(self.open_nodes, start_node)
                self.all_nodes.append(start_node)
                self.all_positions.append(np.array([0, 0], dtype=int))

            self._get_single_path(global_map=global_map, target_positions=target_positions,
                                 start_position=start_position,
                                 tidx=tidx)
        return

    def _A_star(self, start_position, target_position, global_map):
        """
        single start and target position
        """
        current_path = self._check_straight_line(global_map=global_map, target=target_position,
                                                 current_position=start_position)
        if current_path is not None:
            return [start_position]

        path = [np.inf]
        open_nodes = []
        all_nodes = []
        all_positions = []
        idx = 0
        iteration = 0
        start_node = Node(parent=None, position=np.array([0, 0], dtype=int), idx=idx,
                          cost_to_go=np.linalg.norm(start_position - target_position), cost_to_come=0)
        heapq.heappush(open_nodes, start_node)
        all_nodes.append(start_node)
        all_positions.append(np.array([0, 0], dtype=int))
        # heapq.heappush(all_nodes, (idx, start_node))
        while open_nodes and iteration < self.max_iteration:
            node = heapq.heappop(open_nodes)

            children = self._select_grid_action(global_map=global_map, current_position=node.position,
                                                path=all_positions,
                                                start_position=start_position)
            for child_position in children:
                current_real_position = child_position * self.step_size + start_position
                cost_to_go = np.linalg.norm(current_real_position - target_position)
                parent_idx, cost_to_come = self._check_parent_node(open_nodes, child_position, parent_node=node)
                idx += 1
                child_node = Node(parent=parent_idx, position=child_position, idx=idx, cost_to_come=cost_to_come,
                                  cost_to_go=cost_to_go)
                all_positions.append(child_position)
                all_nodes.append(child_node)
                heapq.heappush(open_nodes, child_node)
                if cost_to_go < self.target_arriving_threshold:
                    final_path = self._trace_back(all_nodes=all_nodes, current_node=child_node,
                                                  start_position=start_position)
                    return final_path
            iteration += 1
        print("iteration: {}".format(iteration))
        if len(path) == 1:
            return None
        else:
            return path[1]

    def calculate_paths_separately(self, center, targets, global_map):
        paths = []
        for target in targets:
            path = self._A_star(start_position=center, target_position=target, global_map=global_map)
            if path is None:
                continue
            path.reverse()
            path.append(target)
            paths.append(np.copy(path))
        return paths

    def calculate_paths_together(self, position, targets, global_map):
        self._A_star_all(start_position=position, target_positions=targets, global_map=global_map)
        return self.all_targets_paths

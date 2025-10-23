from turtle import Vec2D

import numpy as np


class Node:
    def __init__(self, x, y, parent=None):
        self.vec = Vec2D(x, y)

        self.parent: Node | None = parent
        self.children: list[Node] = []

    def add_child(self, child):
        child.parent = self
        self.children.append(child)


class RRT:
    def __init__(self, x_start_pose=0, y_start_pose=0, x_goal_pose=0, y_goal_pose=0):
        self.start = Node(x_start_pose, y_start_pose)
        self.goal = Node(x_goal_pose, y_goal_pose)

        self.is_complete = False

        self.root: Node = self.start

    @staticmethod
    def _find_closest_recursive(current_node: Node, target_pos: Vec2D, best_match: dict[str, Node | float]):
        """Helper function to perform the recursive search."""

        dist_sq = abs(current_node.vec - target_pos)
        if dist_sq < best_match['min_dist_sq']:
            best_match['node'] = current_node
            best_match['min_dist_sq'] = dist_sq

        for child in current_node.children:
            RRT._find_closest_recursive(child, target_pos, best_match)

    def find_closest_node(self, target_position):
        """
        Traverses the tree starting from the root to find the node
        closest to the target_position.

        Returns:
            A node that is closest to the target_position.
        """

        initial_dist_sq = abs(self.root.vec - target_position)
        best_match = {
            'node': self.root,
            'min_dist_sq': initial_dist_sq
        }

        self._find_closest_recursive(self.root, target_position, best_match)

        return best_match['node']

"""
Nearest-Neighbor + 2-opt (swap edges) planner node.

Inherits PathPlanningBase:
  - ROS2 I/O is handled by base
  - We only implement compute_tour()
"""

from typing import List, Tuple
import numpy as np

import rclpy

from path_planning.path_planning_base import PathPlanningBase


def tour_cost_closed(tour: List[int], dist_mat: np.ndarray) -> float:
    return float(sum(dist_mat[tour[i], tour[i + 1]] for i in range(len(tour) - 1)))


def nearest_neighbor_tour(dist_mat: np.ndarray, start_idx: int = 0, close_loop: bool = True) -> List[int]:
    n = dist_mat.shape[0]
    unvisited = set(range(n))
    unvisited.remove(start_idx)

    tour = [start_idx]
    cur = start_idx
    while unvisited:
        nxt = min(unvisited, key=lambda j: dist_mat[cur, j])
        if not np.isfinite(dist_mat[cur, nxt]):
            raise ValueError(f"NN found unreachable edge {cur}->{nxt}.")
        tour.append(nxt)
        unvisited.remove(nxt)
        cur = nxt

    if close_loop:
        tour.append(start_idx)
    return tour


def two_opt_greedy_multi_swap(
    tour: List[int],
    dist_mat: np.ndarray,
    first_improvement: bool = True,
    max_iter: int = 10_000,
) -> Tuple[List[int], int]:
    """
    2-opt on a CLOSED tour. (tour[0] == tour[-1] required)
    Swap edges by reversing a segment [i:k].
    """
    if len(tour) < 4 or tour[0] != tour[-1]:
        return tour, 0

    n = len(tour) - 1  # last is same as first
    swaps = 0

    for _ in range(max_iter):
        improved = False

        # keep start fixed -> i starts at 1, and we avoid touching the last '0'
        for i in range(1, n - 1):
            for k in range(i + 1, n):
                a, b = tour[i - 1], tour[i]
                c, d = tour[k], tour[k + 1]

                if not (
                    np.isfinite(dist_mat[a, b]) and np.isfinite(dist_mat[c, d]) and
                    np.isfinite(dist_mat[a, c]) and np.isfinite(dist_mat[b, d])
                ):
                    continue

                delta = (dist_mat[a, c] + dist_mat[b, d]) - (dist_mat[a, b] + dist_mat[c, d])
                if delta < 0:
                    tour[i:k + 1] = reversed(tour[i:k + 1])
                    swaps += 1
                    improved = True
                    if first_improvement:
                        break
            if improved and first_improvement:
                break

        if not improved:
            break

    return tour, swaps


class NN2OptPathPlanningNode(PathPlanningBase):
    def __init__(self):
        super().__init__("path_planning_nn_2opt")

        self.get_logger().info("NN2OptPathPlanningNode started (derived).")

    def declare_algorithm_parameters(self) -> None:
        # 2-opt settings
        self.declare_parameter("two_opt_first_improvement", True)
        self.declare_parameter("two_opt_max_iter", 5000)

        # Note: your original code skipped 2-opt for open tours; we keep that behavior.
        # If you later want 2-opt for open paths, it’s a slightly different variant.

    def compute_tour(self, dist_mat: np.ndarray, close_loop: bool) -> List[int]:
        first_improvement = bool(self.get_parameter("two_opt_first_improvement").value)
        max_iter = int(self.get_parameter("two_opt_max_iter").value)

        # 1) nearest-neighbor tour
        nn_tour = nearest_neighbor_tour(dist_mat, start_idx=0, close_loop=close_loop)

        # 2) 2-opt ONLY if closed loop (swap edges assumes k+1 exists and start=end)
        if close_loop:
            opt_tour, swaps = two_opt_greedy_multi_swap(
                nn_tour.copy(),
                dist_mat,
                first_improvement=first_improvement,
                max_iter=max_iter,
            )
            nn_cost = tour_cost_closed(nn_tour, dist_mat)
            opt_cost = tour_cost_closed(opt_tour, dist_mat)

            self.get_logger().info(
                f"TSP heuristic: NN cost={nn_cost:.2f}, 2-opt cost={opt_cost:.2f}, swaps={swaps}"
            )
            return opt_tour

        # open tour: return NN as-is (matches your earlier behavior)
        open_cost = float(sum(dist_mat[nn_tour[i], nn_tour[i + 1]] for i in range(len(nn_tour) - 1)))
        self.get_logger().info(f"TSP heuristic (open): NN cost={open_cost:.2f}")
        return nn_tour


def main(args=None):
    rclpy.init(args=args)
    node = NN2OptPathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

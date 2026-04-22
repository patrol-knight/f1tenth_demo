"""
TSP Solver with A* Path Planning for Robot Navigation
Uses grid map format: 0 = valid/passable, 1 = obstacle
"""

import numpy as np
import heapq
import time
import itertools
from typing import List, Tuple, Set, Dict, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class AStarPathFinder:
    """A* algorithm for pathfinding with grid-based obstacles"""
    
    def __init__(self, grid_map: np.ndarray):
        """
        Initialize pathfinder with grid map
        Args:
            grid_map: 2D numpy array where 0=valid, 1=obstacle
        """
        self.grid_map = grid_map
        self.height, self.width = grid_map.shape
        
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def is_valid(self, x: int, y: int) -> bool:
        """Check if position is valid (within bounds and not obstacle)"""
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        return self.grid_map[y, x] == 0  # 0 = valid, 1 = obstacle
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring positions (8-directional movement)"""
        x, y = pos
        neighbors = []
        
        # 8-directional movement
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], float]:
        """
        Find shortest path from start to goal using A*
        Returns: (path, distance)
        """
        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            return [], float('inf')
        
        # Priority queue: (f_score, counter, position)
        counter = 0
        open_set = [(0, counter, start)]
        counter += 1
        
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start: 0}
        f_score: Dict[Tuple[int, int], float] = {start: self.heuristic(start, goal)}
        
        open_set_hash: Set[Tuple[int, int]] = {start}
        
        while open_set:
            current_f, _, current = heapq.heappop(open_set)
            open_set_hash.discard(current)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, g_score[goal]
            
            for neighbor in self.get_neighbors(current):
                # Calculate cost (diagonal moves cost sqrt(2), cardinal moves cost 1)
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if (dx + dy) == 2 else 1.0
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                        counter += 1
                        open_set_hash.add(neighbor)
        
        # No path found
        return [], float('inf')


class TSPSolverDP:
    """
    TSP Solver using Dynamic Programming (Held-Karp Algorithm)
    
    Time Complexity: O(n² · 2^n)
    Space Complexity: O(n · 2^n)
    
    Much faster than brute force O(n!) for n > 10
    """
    
    def __init__(self, waypoints: List[Tuple[int, int]], 
                 pathfinder: AStarPathFinder):
        self.waypoints = waypoints
        self.pathfinder = pathfinder
        self.n = len(waypoints)
        self.distance_matrix = self._compute_distance_matrix()
    
    def _compute_distance_matrix(self) -> np.ndarray:
        """Compute pairwise distances between all waypoints using A*"""
        n = len(self.waypoints)
        matrix = np.zeros((n, n))
        
        print("Computing distance matrix using A* pathfinding...")
        for i in range(n):
            for j in range(n):
                if i != j:
                    path, dist = self.pathfinder.find_path(
                        self.waypoints[i], 
                        self.waypoints[j]
                    )
                    matrix[i][j] = dist if path else float('inf')
        
        return matrix
    
    def solve_dp(self) -> Tuple[List[int], float]:
        """
        Solve TSP using Dynamic Programming (Held-Karp algorithm)
        
        DP State:
        dp[mask][i] = minimum cost to visit all cities in mask, ending at city i
        
        mask: bitmask representing set of visited cities
        i: current city (last city visited)
        
        Returns: (best_tour, best_distance)
        """
        n = self.n
        
        # DP table: dp[mask][i] = (min_cost, previous_city)
        # mask: bitmask of visited cities
        # i: current ending city
        dp: Dict[Tuple[int, int], Tuple[float, int]] = {}
        
        # Base case: start from city 0, visit only city 0
        # mask with only bit 0 set = 1
        dp[(1, 0)] = (0, -1)
        
        # Iterate through all subsets (in order of size)
        for mask in range(1, 1 << n):
            # Skip if city 0 is not in the subset
            if not (mask & 1):
                continue
            
            # For each city in the current subset
            for i in range(n):
                if not (mask & (1 << i)):
                    continue
                
                # Try to extend from previous cities
                prev_mask = mask ^ (1 << i)  # Remove city i from mask
                
                if prev_mask == 0:  # Only city i in mask
                    continue
                
                # Try all possible previous cities
                for j in range(n):
                    if not (prev_mask & (1 << j)):
                        continue
                    
                    if (prev_mask, j) not in dp:
                        continue
                    
                    prev_cost = dp[(prev_mask, j)][0]
                    new_cost = prev_cost + self.distance_matrix[j][i]
                    
                    if (mask, i) not in dp or new_cost < dp[(mask, i)][0]:
                        dp[(mask, i)] = (new_cost, j)
        
        # Find the best tour ending at any city, then return to start (city 0)
        full_mask = (1 << n) - 1  # All cities visited
        best_cost = float('inf')
        best_last = -1
        
        for i in range(1, n):  # Don't consider ending at start city
            if (full_mask, i) in dp:
                cost = dp[(full_mask, i)][0] + self.distance_matrix[i][0]
                if cost < best_cost:
                    best_cost = cost
                    best_last = i
        
        if best_last == -1:
            return None, float('inf')
        
        # Reconstruct the path
        tour = self._reconstruct_path(dp, full_mask, best_last)
        
        return tour, best_cost
    
    def _reconstruct_path(self, dp: Dict[Tuple[int, int], Tuple[float, int]], 
                          mask: int, last: int) -> List[int]:
        """Reconstruct the tour from DP table"""
        path = []
        current = last
        current_mask = mask
        
        while current != -1:
            path.append(current)
            if current_mask == 0:
                break
            
            if (current_mask, current) not in dp:
                break
            
            _, prev = dp[(current_mask, current)]
            current_mask ^= (1 << current)  # Remove current city from mask
            current = prev
        
        path.reverse()
        path.append(0)  # Return to start
        
        return path
    
    def solve_brute_force(self) -> Tuple[List[int], float]:
        """
        Brute force TSP for comparison
        Time Complexity: O(n!)
        """
        import itertools
        
        n = self.n
        other_points = list(range(1, n))
        
        best_tour = None
        best_distance = float('inf')
        
        for perm in itertools.permutations(other_points):
            tour = [0] + list(perm) + [0]
            distance = self._calculate_tour_distance(tour)
            
            if distance < best_distance:
                best_distance = distance
                best_tour = tour
        
        return best_tour, best_distance
    
    def _calculate_tour_distance(self, tour: List[int]) -> float:
        """Calculate total distance of a tour"""
        total = 0
        for i in range(len(tour) - 1):
            total += self.distance_matrix[tour[i]][tour[i + 1]]
        return total
    
    def get_complete_path(self, tour: List[int]) -> List[Tuple[int, int]]:
        """Get complete path following the tour using A*"""
        complete_path = []
        
        for i in range(len(tour) - 1):
            path, _ = self.pathfinder.find_path(
                self.waypoints[tour[i]], 
                self.waypoints[tour[i + 1]]
            )
            
            if i == 0:
                complete_path.extend(path)
            else:
                complete_path.extend(path[1:])
        
        return complete_path



def visualize_solution(grid_map: np.ndarray,
                       waypoints: List[Tuple[int, int]], 
                       tour: List[int], 
                       complete_path: List[Tuple[int, int]],
                       title: str,
                       save_path: Optional[str] = None):
    """Visualize the TSP solution with grid map"""
    fig, ax = plt.subplots(figsize=(14, 12))
    
    # Draw grid map (0=white/valid, 1=gray/obstacle)
    height, width = grid_map.shape
    
    # Create color map for visualization
    for y in range(height):
        for x in range(width):
            if grid_map[y, x] == 1:  # Obstacle
                rect = Rectangle((x, y), 1, 1, 
                               facecolor='gray', edgecolor='lightgray', alpha=0.7)
                ax.add_patch(rect)
    
    # Draw grid lines
    for x in range(width + 1):
        ax.axvline(x, color='lightgray', linewidth=0.5, alpha=0.3)
    for y in range(height + 1):
        ax.axhline(y, color='lightgray', linewidth=0.5, alpha=0.3)
    
    # Draw complete path
    if complete_path:
        path_x = [p[0] + 0.5 for p in complete_path]  # Center in cell
        path_y = [p[1] + 0.5 for p in complete_path]
        ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.6, label='A* Path')
    
    # Draw tour connections (dashed lines)
    for i in range(len(tour) - 1):
        start = waypoints[tour[i]]
        end = waypoints[tour[i + 1]]
        ax.plot([start[0] + 0.5, end[0] + 0.5], 
               [start[1] + 0.5, end[1] + 0.5], 
               'g--', linewidth=2.5, alpha=0.4)
    
    # Draw waypoints
    for i, wp in enumerate(waypoints):
        if i == 0:
            ax.plot(wp[0] + 0.5, wp[1] + 0.5, 'ro', markersize=16, 
                   label='Start/End', zorder=5)
        else:
            ax.plot(wp[0] + 0.5, wp[1] + 0.5, 'bs', markersize=13, 
                   label='Waypoint' if i == 1 else '', zorder=5)
        ax.text(wp[0] + 0.5, wp[1] + 1.5, f'{i}', fontsize=11, 
               fontweight='bold', ha='center')
    
    # Draw tour order
    tour_text = ' → '.join(map(str, tour))
    ax.text(0.02, 0.98, f'Tour: {tour_text}', 
           transform=ax.transAxes, fontsize=11, verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.set_aspect('equal')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    plt.tight_layout()
    return fig


def create_grid_map(width: int, height: int, 
                    obstacle_rects: List[Tuple[int, int, int, int]]) -> np.ndarray:
    """
    Create grid map from obstacle rectangles
    Args:
        width, height: Map dimensions
        obstacle_rects: List of (x, y, w, h) tuples defining rectangular obstacles
    Returns:
        2D numpy array where 0=valid, 1=obstacle
    """
    grid = np.zeros((height, width), dtype=int)
    
    for x, y, w, h in obstacle_rects:
        grid[y:y+h, x:x+w] = 1  # Mark as obstacle
    
    return grid


def print_grid_map(grid_map: np.ndarray, waypoints: List[Tuple[int, int]] = None):
    """Print grid map in text format for visualization"""
    height, width = grid_map.shape
    print("\nGrid Map (0=valid, 1=obstacle, W=waypoint):")
    print("┌" + "─" * width + "┐")
    
    for y in range(height):
        row = "│"
        for x in range(width):
            if waypoints and (x, y) in waypoints:
                idx = waypoints.index((x, y))
                row += str(idx) if idx < 10 else "W"
            elif grid_map[y, x] == 1:
                row += "█"
            else:
                row += " "
        row += "│"
        print(row)
    
    print("└" + "─" * width + "┘")


def run_test_case(name: str, 
                  grid_map: np.ndarray,
                  waypoints: List[Tuple[int, int]],
                  use_brute_force: bool = True):
    """Run a complete test case"""
    print(f"\n{'='*70}")
    print(f"TEST CASE: {name}")
    print(f"{'='*70}")
    print(f"Grid size: {grid_map.shape[1]} x {grid_map.shape[0]} (width x height)")
    print(f"Number of waypoints: {len(waypoints)}")
    print(f"Number of obstacle cells: {np.sum(grid_map)}")
    print(f"Waypoints: {waypoints}")
    
    # Print grid map
    print_grid_map(grid_map, waypoints)
    
    # Initialize pathfinder
    pathfinder = AStarPathFinder(grid_map)
    
    # Initialize TSP solver
    tsp_solver = TSPSolverDP(waypoints, pathfinder)
    
    # Solve using selected method
    print(f"\n{'─'*70}")
    if use_brute_force and len(waypoints) <= 10:
        print("Solving TSP using BRUTE FORCE (Exact solution)...")
        start_time = time.time()
        tour, distance = tsp_solver.solve_brute_force()
        solve_time = time.time() - start_time
        method = "Brute Force"
    else:
        print("Solving TSP using NEAREST NEIGHBOR heuristic...")
        start_time = time.time()
        tour, distance = tsp_solver.solve_nearest_neighbor()
        solve_time = time.time() - start_time
        method = "Nearest Neighbor"
    
    print(f"TSP solving time: {solve_time:.6f} seconds")
    print(f"Optimal tour: {tour}")
    print(f"Total tour distance: {distance:.2f} units")
    
    # Get complete path
    print("\nGenerating complete A* path...")
    path_start_time = time.time()
    complete_path = tsp_solver.get_complete_path(tour)
    path_time = time.time() - path_start_time
    
    print(f"A* pathfinding time: {path_time:.6f} seconds")
    print(f"Complete path length: {len(complete_path)} steps")
    
    # Total time
    total_time = solve_time + path_time
    print(f"\n{'─'*70}")
    print(f"SUMMARY:")
    print(f"  Method: {method}")
    print(f"  Tour distance: {distance:.2f} units")
    print(f"  Path steps: {len(complete_path)}")
    print(f"  TSP solve time: {solve_time:.6f} seconds")
    print(f"  A* path time: {path_time:.6f} seconds")
    print(f"  Total time: {total_time:.6f} seconds")
    print(f"{'─'*70}")
    
    # Visualize
    title = f"{name}\nMethod: {method} | Distance: {distance:.2f} | Time: {total_time:.4f}s"
    save_path = f"{name.lower().replace(' ', '_')}.png"
    fig = visualize_solution(grid_map, waypoints, tour, complete_path, title, save_path)
    plt.close(fig)
    
    return {
        'name': name,
        'method': method,
        'tour': tour,
        'distance': distance,
        'path_length': len(complete_path),
        'tsp_time': solve_time,
        'astar_time': path_time,
        'total_time': total_time
    }


def main():
    """Main function to run all test cases"""
    print("="*70)
    print("TSP SOLVER WITH A* PATH PLANNING (Grid Map Format)")
    print("Grid Map: 0 = valid/passable, 1 = obstacle")
    print("="*70)
    
    results = []
    
    # ========================================================================
    # TEST CASE 1: Small warehouse with scattered obstacles
    # ========================================================================
    print("\n" + "="*70)
    print("Setting up Test Case 1...")
    print("="*70)
    
    # Create grid map
    obstacle_rects_1 = [
        (15, 15, 8, 8),   # (x, y, width, height)
        (30, 10, 6, 15),
        (20, 30, 10, 6),
        (35, 30, 8, 8),
    ]
    grid_map_1 = create_grid_map(50, 50, obstacle_rects_1)
    
    waypoints_1 = [
        (5, 5),    # Start/End (depot)
        (45, 10),  # Waypoint 1
        (40, 45),  # Waypoint 2
        (10, 40),  # Waypoint 3
        (25, 25),  # Waypoint 4
    ]
    
    result_1 = run_test_case(
        "Test Case 1 - Small Warehouse",
        grid_map_1, waypoints_1, use_brute_force=True
    )
    results.append(result_1)
    
    # ========================================================================
    # TEST CASE 2: Medium factory floor with corridor obstacles
    # ========================================================================
    print("\n" + "="*70)
    print("Setting up Test Case 2...")
    print("="*70)
    
    obstacle_rects_2 = [
        (20, 10, 3, 35),   # Vertical wall
        (35, 20, 3, 30),   # Vertical wall
        (55, 15, 3, 30),   # Vertical wall
        (25, 45, 20, 3),   # Horizontal wall
        (45, 25, 15, 3),   # Horizontal wall
    ]
    grid_map_2 = create_grid_map(80, 60, obstacle_rects_2)
    
    waypoints_2 = [
        (5, 5),     # Start/End
        (75, 10),   # Waypoint 1
        (70, 55),   # Waypoint 2
        (40, 50),   # Waypoint 3
        (15, 55),   # Waypoint 4
        (15, 30),   # Waypoint 5
        (50, 30),   # Waypoint 6
    ]
    
    result_2 = run_test_case(
        "Test Case 2 - Factory Floor",
        grid_map_2, waypoints_2, use_brute_force=True
    )
    results.append(result_2)
    
    # ========================================================================
    # TEST CASE 3: Large distribution center with complex obstacles
    # ========================================================================
    print("\n" + "="*70)
    print("Setting up Test Case 3...")
    print("="*70)
    
    obstacle_rects_3 = [
        # Storage racks
        (30, 15, 15, 6),
        (30, 25, 15, 6),
        (50, 15, 15, 6),
        (50, 25, 15, 6),
        (70, 15, 10, 6),
        # Loading area
        (15, 50, 8, 15),
        (35, 55, 20, 8),
        # Central pillars
        (48, 45, 4, 4),
        (65, 60, 6, 6),
    ]
    grid_map_3 = create_grid_map(100, 80, obstacle_rects_3)
    
    waypoints_3 = [
        (10, 10),   # Start/End (depot)
        (90, 15),   # Waypoint 1
        (85, 70),   # Waypoint 2
        (50, 70),   # Waypoint 3
        (20, 65),   # Waypoint 4
        (25, 40),   # Waypoint 5
        (60, 40),   # Waypoint 6
        (75, 50),   # Waypoint 7
        (45, 20),   # Waypoint 8
    ]
    
    result_3 = run_test_case(
        "Test Case 3 - Distribution Center",
        grid_map_3, waypoints_3, use_brute_force=True
    )
    results.append(result_3)
    
    # ========================================================================
    # FINAL SUMMARY
    # ========================================================================
    print("\n" + "="*70)
    print("FINAL SUMMARY - ALL TEST CASES")
    print("="*70)
    print(f"{'Test Case':<30} {'Method':<18} {'Distance':<12} {'Steps':<8} {'Time (s)':<10}")
    print("-"*70)
    
    for r in results:
        print(f"{r['name']:<30} {r['method']:<18} {r['distance']:<12.2f} {r['path_length']:<8} {r['total_time']:<10.6f}")
    
    print("="*70)
    print("\nVisualization files saved:")
    for r in results:
        filename = r['name'].lower().replace(' ', '_') + '.png'
        print(f"  - {filename}")
    
    print("\n✓ All test cases completed successfully!")
    print("\nGrid Map Format: 0 = valid/passable, 1 = obstacle")


if __name__ == "__main__":
    main()
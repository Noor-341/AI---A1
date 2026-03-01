"""
Dynamic Pathfinding Agent
AI Assignment 2 - Spring 2026
Single File Implementation using Pygame
"""

import pygame
import random
import math
import heapq
import time
from enum import Enum

# ==================== CONFIGURATION ====================
# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
LIGHT_GRAY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)

# Grid settings
CELL_SIZE = 30
INFO_PANEL_WIDTH = 300
WINDOW_PADDING = 10

# Dynamic obstacle settings
OBSTACLE_SPAWN_PROB = 0.02  # 2% chance per time step
REPLAN_CHECK_INTERVAL = 10  # Check for path blockage every 10 frames

# Cell Types
class CellType(Enum):
    EMPTY = 0
    OBSTACLE = 1
    START = 2
    GOAL = 3
    VISITED = 4
    FRONTIER = 5
    PATH = 6

# ==================== NODE CLASS ====================
class Node:
    """Represents a node in the grid for pathfinding"""
    def __init__(self, x, y, cell_type=CellType.EMPTY):
        self.x = x
        self.y = y
        self.cell_type = cell_type
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.parent = None
        
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

# ==================== GRID CLASS ====================
class Grid:
    """Manages the grid and its cells"""
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = [[Node(x, y) for y in range(cols)] for x in range(rows)]
        self.start = None
        self.goal = None
        self.obstacle_density = 0.3
        
    def reset_cells_state(self):
        """Reset all cells to their base state (keep obstacles, start, goal)"""
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j].cell_type not in [CellType.START, CellType.GOAL, CellType.OBSTACLE]:
                    self.grid[i][j].cell_type = CellType.EMPTY
                self.grid[i][j].g_cost = float('inf')
                self.grid[i][j].h_cost = 0
                self.grid[i][j].f_cost = float('inf')
                self.grid[i][j].parent = None
    
    def set_start(self, x, y):
        """Set the start position"""
        if self.start:
            self.start.cell_type = CellType.EMPTY
        self.start = self.grid[x][y]
        self.start.cell_type = CellType.START
    
    def set_goal(self, x, y):
        """Set the goal position"""
        if self.goal:
            self.goal.cell_type = CellType.EMPTY
        self.goal = self.grid[x][y]
        self.goal.cell_type = CellType.GOAL
    
    def toggle_obstacle(self, x, y):
        """Toggle obstacle at given position"""
        if self.grid[x][y].cell_type == CellType.OBSTACLE:
            self.grid[x][y].cell_type = CellType.EMPTY
        elif self.grid[x][y].cell_type == CellType.EMPTY:
            self.grid[x][y].cell_type = CellType.OBSTACLE
    
    def random_generate(self, density=None):
        """Randomly generate obstacles"""
        if density is not None:
            self.obstacle_density = density
        
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j].cell_type not in [CellType.START, CellType.GOAL]:
                    if random.random() < self.obstacle_density:
                        self.grid[i][j].cell_type = CellType.OBSTACLE
                    else:
                        self.grid[i][j].cell_type = CellType.EMPTY
    
    def spawn_random_obstacle(self):
        """Spawn a random obstacle for dynamic mode"""
        if not self.start or not self.goal:
            return
        
        x = random.randint(0, self.rows - 1)
        y = random.randint(0, self.cols - 1)
        
        # Don't place on start or goal
        if (x, y) != (self.start.x, self.start.y) and (x, y) != (self.goal.x, self.goal.y):
            if self.grid[x][y].cell_type == CellType.EMPTY:
                self.grid[x][y].cell_type = CellType.OBSTACLE
                return (x, y)
        return None
    
    def get_neighbors(self, node, allow_diagonal=False):
        """Get valid neighbors of a node"""
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-directional movement
        
        if allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols:
                if self.grid[nx][ny].cell_type != CellType.OBSTACLE:
                    neighbors.append(self.grid[nx][ny])
        
        return neighbors
    
    def get_move_cost(self, current, neighbor):
        """Calculate movement cost between adjacent nodes"""
        # Diagonal movement costs more
        if abs(current.x - neighbor.x) == 1 and abs(current.y - neighbor.y) == 1:
            return 1.41  # sqrt(2)
        return 1.0

# ==================== SEARCH ALGORITHMS ====================
class SearchAlgorithms:
    """Implements various search algorithms"""
    
    @staticmethod
    def manhattan_distance(node1, node2):
        """Manhattan distance heuristic"""
        return abs(node1.x - node2.x) + abs(node1.y - node2.y)
    
    @staticmethod
    def euclidean_distance(node1, node2):
        """Euclidean distance heuristic"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    @staticmethod
    def greedy_bfs(grid, start, goal, heuristic_func, metrics):
        """Greedy Best-First Search Algorithm"""
        metrics['nodes_visited'] = 0
        metrics['execution_time'] = 0
        start_time = time.time()
        
        # Priority queue: (f_cost, node)
        frontier = []
        heapq.heappush(frontier, (0, id(start), start))
        
        # Visited set (strict visited list)
        visited = set()
        visited.add(start)
        
        # Parent dictionary for path reconstruction
        parent = {start: None}
        
        while frontier:
            _, _, current = heapq.heappop(frontier)
            current.cell_type = CellType.VISITED
            metrics['nodes_visited'] += 1
            
            if current == goal:
                # Goal reached
                path = SearchAlgorithms.reconstruct_path(parent, start, goal)
                metrics['execution_time'] = (time.time() - start_time) * 1000  # in ms
                metrics['path_cost'] = SearchAlgorithms.calculate_path_cost(grid, path)
                return path
            
            for neighbor in grid.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    neighbor.parent = current
                    parent[neighbor] = current
                    neighbor.cell_type = CellType.FRONTIER
                    
                    # f(n) = h(n) for GBFS
                    h_cost = heuristic_func(neighbor, goal)
                    heapq.heappush(frontier, (h_cost, id(neighbor), neighbor))
        
        metrics['execution_time'] = (time.time() - start_time) * 1000
        return None
    
    @staticmethod
    def astar(grid, start, goal, heuristic_func, metrics):
        """A* Search Algorithm"""
        metrics['nodes_visited'] = 0
        metrics['execution_time'] = 0
        start_time = time.time()
        
        # Priority queue: (f_cost, node)
        frontier = []
        heapq.heappush(frontier, (0, id(start), start))
        
        # Track nodes in frontier for quick lookup
        frontier_dict = {start: 0}
        
        # Visited/expanded set (closed list)
        closed_set = set()
        
        # Initialize start node costs
        start.g_cost = 0
        start.h_cost = heuristic_func(start, goal)
        start.f_cost = start.g_cost + start.h_cost
        
        while frontier:
            _, _, current = heapq.heappop(frontier)
            
            if current in frontier_dict:
                del frontier_dict[current]
            
            closed_set.add(current)
            current.cell_type = CellType.VISITED
            metrics['nodes_visited'] += 1
            
            if current == goal:
                # Goal reached
                path = SearchAlgorithms.reconstruct_path_parents(current)
                metrics['execution_time'] = (time.time() - start_time) * 1000
                metrics['path_cost'] = SearchAlgorithms.calculate_path_cost(grid, path)
                return path
            
            for neighbor in grid.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g cost
                tentative_g = current.g_cost + grid.get_move_cost(current, neighbor)
                
                if neighbor not in frontier_dict or tentative_g < neighbor.g_cost:
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g
                    neighbor.h_cost = heuristic_func(neighbor, goal)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    if neighbor not in frontier_dict:
                        heapq.heappush(frontier, (neighbor.f_cost, id(neighbor), neighbor))
                        frontier_dict[neighbor] = neighbor.f_cost
                        neighbor.cell_type = CellType.FRONTIER
        
        metrics['execution_time'] = (time.time() - start_time) * 1000
        return None
    
    @staticmethod
    def reconstruct_path(parents, start, goal):
        """Reconstruct path from parent dictionary"""
        path = []
        current = goal
        while current:
            path.append(current)
            current = parents.get(current)
        path.reverse()
        return path
    
    @staticmethod
    def reconstruct_path_parents(node):
        """Reconstruct path from node parents"""
        path = []
        current = node
        while current:
            path.append(current)
            current = current.parent
        path.reverse()
        return path
    
    @staticmethod
    def calculate_path_cost(grid, path):
        """Calculate total cost of a path"""
        if not path:
            return 0
        
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost += grid.get_move_cost(path[i], path[i + 1])
        return total_cost

# ==================== BUTTON CLASS ====================
class Button:
    """Simple button class for GUI"""
    def __init__(self, x, y, width, height, text, color=LIGHT_GRAY, text_color=BLACK):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.text_color = text_color
        self.clicked = False
        
    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        
        font = pygame.font.Font(None, 24)
        text_surface = font.render(self.text, True, self.text_color)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)
        
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.clicked = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.clicked = False
        return False

# ==================== MAIN APPLICATION ====================
class PathfindingApp:
    """Main application class"""
    def __init__(self):
        pygame.init()
        
        # Initial grid size
        self.rows = 20
        self.cols = 20
        
        # Calculate window size
        self.grid_width = self.cols * CELL_SIZE
        self.window_width = self.grid_width + INFO_PANEL_WIDTH + WINDOW_PADDING * 2
        self.window_height = max(self.rows * CELL_SIZE + WINDOW_PADDING * 2, 600)
        
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("Dynamic Pathfinding Agent - AI Assignment 2")
        
        self.clock = pygame.time.Clock()
        self.running = True
        
        # Initialize grid
        self.grid = Grid(self.rows, self.cols)
        
        # Set default start and goal
        self.grid.set_start(0, 0)
        self.grid.set_goal(self.rows - 1, self.cols - 1)
        
        # Algorithm selection
        self.current_algorithm = 'astar'  # 'gbfs' or 'astar'
        self.current_heuristic = 'manhattan'  # 'manhattan' or 'euclidean'
        
        # Mode
        self.dynamic_mode = False
        self.auto_replan = True
        
        # Path and metrics
        self.current_path = None
        self.metrics = {
            'nodes_visited': 0,
            'path_cost': 0,
            'execution_time': 0
        }
        
        # Dynamic mode variables
        self.frame_count = 0
        
        # Create buttons
        self.create_buttons()
        
    def create_buttons(self):
        """Create UI buttons"""
        button_width = INFO_PANEL_WIDTH - 20
        button_height = 40
        x = self.grid_width + WINDOW_PADDING * 2
        y = WINDOW_PADDING
        
        self.buttons = []
        
        # Algorithm buttons
        self.buttons.append(Button(x, y, button_width, button_height, 
                                   "Algorithm: A*" if self.current_algorithm == 'astar' else "Algorithm: GBFS"))
        y += button_height + 5
        
        # Heuristic buttons
        self.buttons.append(Button(x, y, button_width, button_height,
                                   "Heuristic: Manhattan" if self.current_heuristic == 'manhattan' else "Heuristic: Euclidean"))
        y += button_height + 5
        
        # Action buttons
        self.buttons.append(Button(x, y, button_width, button_height, "Find Path"))
        y += button_height + 5
        
        self.buttons.append(Button(x, y, button_width, button_height, "Random Generate"))
        y += button_height + 5
        
        self.buttons.append(Button(x, y, button_width, button_height, "Clear Path"))
        y += button_height + 5
        
        self.buttons.append(Button(x, y, button_width, button_height, "Reset Grid"))
        y += button_height + 5
        
        # Dynamic mode toggle
        self.dynamic_button = Button(x, y, button_width, button_height, 
                                     "Dynamic: OFF", color=RED, text_color=WHITE)
        self.buttons.append(self.dynamic_button)
        y += button_height + 5
        
        # Density slider info
        self.density_y = y
        y += 40
        
        # Metrics display area
        self.metrics_y = y
        
    def get_heuristic_func(self):
        """Get the current heuristic function"""
        if self.current_heuristic == 'manhattan':
            return SearchAlgorithms.manhattan_distance
        else:
            return SearchAlgorithms.euclidean_distance
    
    def find_path(self):
        """Find path using current algorithm"""
        # Reset cell states (keep obstacles, start, goal)
        self.grid.reset_cells_state()
        
        # Reset metrics
        self.metrics = {'nodes_visited': 0, 'path_cost': 0, 'execution_time': 0}
        
        # Run selected algorithm
        heuristic_func = self.get_heuristic_func()
        
        if self.current_algorithm == 'gbfs':
            self.current_path = SearchAlgorithms.greedy_bfs(
                self.grid, self.grid.start, self.grid.goal, heuristic_func, self.metrics
            )
        else:  # astar
            self.current_path = SearchAlgorithms.astar(
                self.grid, self.grid.start, self.grid.goal, heuristic_func, self.metrics
            )
        
        # Mark path
        if self.current_path:
            for node in self.current_path:
                if node not in [self.grid.start, self.grid.goal]:
                    node.cell_type = CellType.PATH
    
    def check_path_blocked(self):
        """Check if current path is blocked by new obstacles"""
        if not self.current_path:
            return False
        
        for node in self.current_path:
            if node != self.grid.start and node != self.grid.goal:
                if node.cell_type == CellType.OBSTACLE:
                    return True
        return False
    
    def handle_click(self, pos):
        """Handle mouse clicks on grid"""
        x, y = pos
        
        # Check if click is on grid
        if x < self.grid_width and y < self.rows * CELL_SIZE:
            grid_x = y // CELL_SIZE
            grid_y = x // CELL_SIZE
            
            if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                # Shift+click to set start
                self.grid.set_start(grid_x, grid_y)
            elif pygame.key.get_mods() & pygame.KMOD_CTRL:
                # Ctrl+click to set goal
                self.grid.set_goal(grid_x, grid_y)
            else:
                # Regular click to toggle obstacle
                self.grid.toggle_obstacle(grid_x, grid_y)
            
            # Clear current path when grid is modified
            self.current_path = None
            self.grid.reset_cells_state()
    
    def run(self):
        """Main game loop"""
        while self.running:
            dt = self.clock.tick(60)
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        # Check button clicks
                        button_clicked = False
                        for i, button in enumerate(self.buttons):
                            if button.handle_event(event):
                                button_clicked = True
                                if i == 0:  # Algorithm toggle
                                    self.current_algorithm = 'astar' if self.current_algorithm == 'gbfs' else 'gbfs'
                                    button.text = f"Algorithm: {'A*' if self.current_algorithm == 'astar' else 'GBFS'}"
                                    self.current_path = None
                                elif i == 1:  # Heuristic toggle
                                    self.current_heuristic = 'manhattan' if self.current_heuristic == 'euclidean' else 'euclidean'
                                    button.text = f"Heuristic: {'Manhattan' if self.current_heuristic == 'manhattan' else 'Euclidean'}"
                                    self.current_path = None
                                elif i == 2:  # Find Path
                                    self.find_path()
                                elif i == 3:  # Random Generate
                                    self.grid.random_generate(self.grid.obstacle_density)
                                    self.current_path = None
                                elif i == 4:  # Clear Path
                                    self.current_path = None
                                    self.grid.reset_cells_state()
                                elif i == 5:  # Reset Grid
                                    self.grid = Grid(self.rows, self.cols)
                                    self.grid.set_start(0, 0)
                                    self.grid.set_goal(self.rows - 1, self.cols - 1)
                                    self.current_path = None
                                elif i == 6:  # Dynamic mode toggle
                                    self.dynamic_mode = not self.dynamic_mode
                                    button.text = f"Dynamic: {'ON' if self.dynamic_mode else 'OFF'}"
                                    button.color = GREEN if self.dynamic_mode else RED
                        
                        # Handle grid click if no button was clicked
                        if not button_clicked:
                            self.handle_click(event.pos)
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.find_path()
                    elif event.key == pygame.K_r:
                        self.grid.random_generate(self.grid.obstacle_density)
                        self.current_path = None
                    elif event.key == pygame.K_c:
                        self.current_path = None
                        self.grid.reset_cells_state()
            
            # Dynamic mode obstacle spawning
            if self.dynamic_mode:
                self.frame_count += 1
                if random.random() < OBSTACLE_SPAWN_PROB:
                    self.grid.spawn_random_obstacle()
                
                # Check if path is blocked and replan
                if self.auto_replan and self.current_path and self.frame_count % REPLAN_CHECK_INTERVAL == 0:
                    if self.check_path_blocked():
                        self.find_path()
            
            # Draw everything
            self.draw()
            pygame.display.flip()
        
        pygame.quit()
    
    def draw(self):
        """Draw everything on screen"""
        self.screen.fill(WHITE)
        
        # Draw grid
        for i in range(self.rows):
            for j in range(self.cols):
                cell = self.grid.grid[i][j]
                rect = pygame.Rect(j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE - 1, CELL_SIZE - 1)
                
                # Choose color based on cell type
                if cell.cell_type == CellType.OBSTACLE:
                    color = BLACK
                elif cell.cell_type == CellType.START:
                    color = GREEN
                elif cell.cell_type == CellType.GOAL:
                    color = RED
                elif cell.cell_type == CellType.VISITED:
                    color = BLUE
                elif cell.cell_type == CellType.FRONTIER:
                    color = YELLOW
                elif cell.cell_type == CellType.PATH:
                    color = CYAN
                else:
                    color = WHITE
                
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, GRAY, rect, 1)  # Grid lines
        
        # Draw info panel
        panel_x = self.grid_width + WINDOW_PADDING
        panel_width = INFO_PANEL_WIDTH - WINDOW_PADDING
        
        # Panel background
        pygame.draw.rect(self.screen, LIGHT_GRAY, 
                        (panel_x, WINDOW_PADDING, panel_width, self.window_height - WINDOW_PADDING * 2))
        
        # Draw buttons
        for button in self.buttons:
            button.draw(self.screen)
        
        # Draw obstacle density slider info
        font = pygame.font.Font(None, 24)
        density_text = font.render(f"Obstacle Density: {int(self.grid.obstacle_density * 100)}%", True, BLACK)
        self.screen.blit(density_text, (panel_x + 10, self.density_y))
        
        # Draw metrics
        metrics_font = pygame.font.Font(None, 22)
        y = self.metrics_y
        
        metrics_title = metrics_font.render("METRICS:", True, BLACK)
        self.screen.blit(metrics_title, (panel_x + 10, y))
        y += 25
        
        nodes_text = metrics_font.render(f"Nodes Visited: {self.metrics['nodes_visited']}", True, BLACK)
        self.screen.blit(nodes_text, (panel_x + 20, y))
        y += 20
        
        cost_text = metrics_font.render(f"Path Cost: {self.metrics['path_cost']:.2f}", True, BLACK)
        self.screen.blit(cost_text, (panel_x + 20, y))
        y += 20
        
        time_text = metrics_font.render(f"Exec Time: {self.metrics['execution_time']:.2f} ms", True, BLACK)
        self.screen.blit(time_text, (panel_x + 20, y))
        y += 30
        
        # Instructions
        inst_font = pygame.font.Font(None, 20)
        inst_y = self.window_height - 180
        
        inst_title = inst_font.render("INSTRUCTIONS:", True, BLACK)
        self.screen.blit(inst_title, (panel_x + 10, inst_y))
        inst_y += 20
        
        inst1 = inst_font.render("Click: Toggle obstacle", True, BLACK)
        self.screen.blit(inst1, (panel_x + 15, inst_y))
        inst_y += 18
        
        inst2 = inst_font.render("Shift+Click: Set Start", True, BLACK)
        self.screen.blit(inst2, (panel_x + 15, inst_y))
        inst_y += 18
        
        inst3 = inst_font.render("Ctrl+Click: Set Goal", True, BLACK)
        self.screen.blit(inst3, (panel_x + 15, inst_y))
        inst_y += 18
        
        inst4 = inst_font.render("Space: Find Path", True, BLACK)
        self.screen.blit(inst4, (panel_x + 15, inst_y))
        inst_y += 18
        
        inst5 = inst_font.render("R: Random Generate", True, BLACK)
        self.screen.blit(inst5, (panel_x + 15, inst_y))
        inst_y += 18
        
        inst6 = inst_font.render("C: Clear Path", True, BLACK)
        self.screen.blit(inst6, (panel_x + 15, inst_y))

# ==================== MAIN ====================
def main():
    """Main entry point"""
    print("=" * 60)
    print("Dynamic Pathfinding Agent - AI Assignment 2")
    print("Spring 2026")
    print("=" * 60)
    print("\nControls:")
    print("  - Click: Toggle obstacle")
    print("  - Shift+Click: Set start position (Green)")
    print("  - Ctrl+Click: Set goal position (Red)")
    print("  - Space: Find path")
    print("  - R: Random generate obstacles")
    print("  - C: Clear path")
    print("\nAlgorithms:")
    print("  - GBFS: Greedy Best-First Search (f(n) = h(n))")
    print("  - A*: A* Search (f(n) = g(n) + h(n))")
    print("\nHeuristics:")
    print("  - Manhattan: |x1-x2| + |y1-y2|")
    print("  - Euclidean: sqrt((x1-x2)^2 + (y1-y2)^2)")
    print("\nDynamic Mode:")
    print("  - Obstacles spawn randomly while moving")
    print("  - Auto-replan when path is blocked")
    print("=" * 60)
    
    app = PathfindingApp()
    app.run()

if __name__ == "__main__":
    main()
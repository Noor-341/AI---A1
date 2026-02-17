"""
AI 2002 - Assignment 1 (Question 7)
Uninformed Search Algorithms in Grid Environment
Single File Implementation - NO DYNAMIC OBSTACLES
"""

import pygame
import sys
from collections import deque
import heapq

# ==================== CONSTANTS ====================
GRID_SIZE = 10
CELL_SIZE = 50
WINDOW_WIDTH = GRID_SIZE * CELL_SIZE
WINDOW_HEIGHT = GRID_SIZE * CELL_SIZE + 80  # Extra space for info
FPS = 5  # Speed of visualization

# Colors
COLORS = {
    'empty': (255, 255, 255),     # White
    'wall': (0, 0, 0),             # Black
    'start': (0, 255, 0),          # Green
    'target': (255, 0, 0),         # Red
    'explored': (173, 216, 230),   # Light Blue
    'frontier': (255, 255, 0),     # Yellow
    'path': (0, 255, 255),         # Cyan
    'grid_line': (200, 200, 200),  # Light Gray
    'text': (0, 0, 0),             # Black
    'panel': (240, 240, 240)       # Light Gray
}

# Movement order: Up, Right, Bottom, Bottom-Right, Left, Top-Left, Top-Right, Bottom-Left
DIRECTIONS = [
    (-1, 0),   # Up
    (0, 1),    # Right
    (1, 0),    # Bottom
    (1, 1),    # Bottom-Right
    (0, -1),   # Left
    (-1, -1),  # Top-Left
    (-1, 1),   # Top-Right
    (1, -1)    # Bottom-Left
]


# ==================== GRID CLASS ====================
class Grid:
    def __init__(self, size):
        self.size = size
        self.grid = [[0 for _ in range(size)] for _ in range(size)]  # 0: empty, 1: wall, 2: start, 3: target
        self.start = (0, 0)
        self.target = (size-1, size-1)
        self.explored = set()
        self.frontier = []
        self.path = []
        self.current_path = []
        
        # Set start and target
        self.set_cell(self.start, 2)
        self.set_cell(self.target, 3)
        
        # Add static walls
        self._add_static_walls()
    
    def _add_static_walls(self):
        """Add some initial walls"""
        walls = [(3,3), (3,4), (4,3), (4,4), 
                 (7,2), (7,3), (8,2), (8,3),
                 (2,7), (2,8), (3,7), (3,8)]
        for x, y in walls:
            if (x, y) != self.start and (x, y) != self.target:
                self.set_cell((x, y), 1)
    
    def set_cell(self, pos, value):
        """Set cell value"""
        x, y = pos
        if 0 <= x < self.size and 0 <= y < self.size:
            self.grid[x][y] = value
    
    def get_cell(self, pos):
        """Get cell value"""
        x, y = pos
        if 0 <= x < self.size and 0 <= y < self.size:
            return self.grid[x][y]
        return -1  # Invalid
    
    def is_valid(self, pos):
        """Check if position is valid and not a wall"""
        x, y = pos
        return (0 <= x < self.size and 
                0 <= y < self.size and 
                self.grid[x][y] != 1)
    
    def get_neighbors(self, pos):
        """Get valid neighbors in specified clockwise order"""
        x, y = pos
        neighbors = []
        
        for dx, dy in DIRECTIONS:
            new_x, new_y = x + dx, y + dy
            new_pos = (new_x, new_y)
            if self.is_valid(new_pos):
                neighbors.append(new_pos)
        
        return neighbors
    
    def reset_search(self):
        """Reset search state"""
        self.explored = set()
        self.frontier = []
        self.path = []
        self.current_path = []
    
    def is_start(self, pos):
        return pos == self.start
    
    def is_target(self, pos):
        return pos == self.target
    
    def is_wall(self, pos):
        x, y = pos
        return self.grid[x][y] == 1


# ==================== ALGORITHMS ====================
def bfs_search(grid):
    """Breadth-First Search"""
    queue = deque([(grid.start, [grid.start])])
    grid.frontier = [grid.start]
    
    while queue:
        node, path = queue.popleft()
        if node in grid.frontier:
            grid.frontier.remove(node)
        
        if node not in grid.explored:
            grid.explored.add(node)
            grid.current_path = path
            
            if node == grid.target:
                grid.path = path
                print(f"✅ BFS: Path found! Length: {len(path)}")
                return
            
            for neighbor in grid.get_neighbors(node):
                if neighbor not in grid.explored and neighbor not in grid.frontier:
                    queue.append((neighbor, path + [neighbor]))
                    grid.frontier.append(neighbor)
            
            yield


def dfs_search(grid):
    """Depth-First Search"""
    stack = [(grid.start, [grid.start])]
    grid.frontier = [grid.start]
    
    while stack:
        node, path = stack.pop()
        if node in grid.frontier:
            grid.frontier.remove(node)
        
        if node not in grid.explored:
            grid.explored.add(node)
            grid.current_path = path
            
            if node == grid.target:
                grid.path = path
                print(f"✅ DFS: Path found! Length: {len(path)}")
                return
            
            # Add neighbors in reverse for proper DFS order
            neighbors = grid.get_neighbors(node)
            for neighbor in reversed(neighbors):
                if neighbor not in grid.explored and neighbor not in grid.frontier:
                    stack.append((neighbor, path + [neighbor]))
                    grid.frontier.append(neighbor)
            
            yield


def ucs_search(grid):
    """Uniform-Cost Search"""
    pq = [(0, grid.start, [grid.start])]  # (cost, node, path)
    grid.frontier = [grid.start]
    costs = {grid.start: 0}
    
    while pq:
        cost, node, path = heapq.heappop(pq)
        
        if node in grid.frontier:
            grid.frontier.remove(node)
        
        if node not in grid.explored:
            grid.explored.add(node)
            grid.current_path = path
            
            if node == grid.target:
                grid.path = path
                print(f"✅ UCS: Path found! Cost: {cost}")
                return
            
            for neighbor in grid.get_neighbors(node):
                if neighbor not in grid.explored:
                    new_cost = cost + 1
                    if neighbor not in costs or new_cost < costs[neighbor]:
                        costs[neighbor] = new_cost
                        if neighbor not in grid.frontier:
                            grid.frontier.append(neighbor)
                        heapq.heappush(pq, (new_cost, neighbor, path + [neighbor]))
            
            yield


def dls_search(grid, limit=10):
    """Depth-Limited Search"""
    
    def dls_recursive(node, path, depth):
        if depth > limit:
            return None
        
        if node not in grid.explored:
            grid.explored.add(node)
            grid.current_path = path
            
            if node == grid.target:
                grid.path = path
                print(f"✅ DLS: Path found! Depth: {depth}")
                return path
            
            if depth < limit:
                for neighbor in grid.get_neighbors(node):
                    if neighbor not in grid.explored and neighbor not in grid.frontier:
                        grid.frontier.append(neighbor)
                        result = yield from dls_recursive(neighbor, path + [neighbor], depth + 1)
                        if result:
                            return result
                        if neighbor in grid.frontier:
                            grid.frontier.remove(neighbor)
            
            yield
    
    grid.frontier = [grid.start]
    yield from dls_recursive(grid.start, [grid.start], 0)


def iddfs_search(grid):
    """Iterative Deepening DFS"""
    max_depth = grid.size * grid.size
    
    for depth in range(1, max_depth + 1):
        print(f"🔄 IDDFS: Trying depth limit {depth}")
        grid.reset_search()
        
        def iddfs_recursive(node, path, current_depth):
            if current_depth > depth:
                return None
            
            if node not in grid.explored:
                grid.explored.add(node)
                grid.current_path = path
                
                if node == grid.target:
                    grid.path = path
                    print(f"✅ IDDFS: Path found at depth {depth}!")
                    return path
                
                if current_depth < depth:
                    for neighbor in grid.get_neighbors(node):
                        if neighbor not in grid.explored and neighbor not in grid.frontier:
                            grid.frontier.append(neighbor)
                            result = yield from iddfs_recursive(neighbor, path + [neighbor], current_depth + 1)
                            if result:
                                return result
                            if neighbor in grid.frontier:
                                grid.frontier.remove(neighbor)
                
                yield
        
        grid.frontier = [grid.start]
        result = yield from iddfs_recursive(grid.start, [grid.start], 0)
        if result:
            return


def bidirectional_search(grid):
    """Bidirectional Search"""
    forward_queue = deque([(grid.start, [grid.start])])
    backward_queue = deque([(grid.target, [grid.target])])
    
    forward_visited = {grid.start: [grid.start]}
    backward_visited = {grid.target: [grid.target]}
    
    grid.frontier = [grid.start, grid.target]
    
    while forward_queue and backward_queue:
        # Forward step
        if forward_queue:
            node, path = forward_queue.popleft()
            if node in grid.frontier:
                grid.frontier.remove(node)
            
            if node not in grid.explored:
                grid.explored.add(node)
                grid.current_path = path
                
                # Check if meeting point found
                if node in backward_visited:
                    meeting_path = path + backward_visited[node][-2::-1]
                    grid.path = meeting_path
                    print(f"✅ Bidirectional: Path found! Length: {len(meeting_path)}")
                    return
                
                for neighbor in grid.get_neighbors(node):
                    if neighbor not in forward_visited and neighbor not in grid.frontier:
                        forward_visited[neighbor] = path + [neighbor]
                        forward_queue.append((neighbor, path + [neighbor]))
                        if neighbor not in grid.frontier:
                            grid.frontier.append(neighbor)
                
                yield
        
        # Backward step
        if backward_queue:
            node, path = backward_queue.popleft()
            
            # Check if meeting point found
            if node in forward_visited:
                meeting_path = forward_visited[node] + path[-2::-1]
                grid.path = meeting_path
                print(f"✅ Bidirectional: Path found! Length: {len(meeting_path)}")
                return
            
            for neighbor in grid.get_neighbors(node):
                if neighbor not in backward_visited and neighbor not in grid.frontier:
                    backward_visited[neighbor] = path + [neighbor]
                    backward_queue.append((neighbor, path + [neighbor]))
                    if neighbor not in grid.frontier:
                        grid.frontier.append(neighbor)
            
            yield


# ==================== GUI CLASS ====================
class GUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("GOOD PERFORMANCE TIME APP")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        self.grid = Grid(GRID_SIZE)
        self.algorithms = [
            ('BFS', bfs_search),
            ('DFS', dfs_search),
            ('UCS', ucs_search),
            ('DLS', dls_search),
            ('IDDFS', iddfs_search),
            ('Bidirectional', bidirectional_search)
        ]
        self.current_algo = 0
        self.searching = False
        self.search_generator = None
        
    def draw_grid(self):
        """Draw the grid and cells"""
        for i in range(self.grid.size):
            for j in range(self.grid.size):
                x = j * CELL_SIZE
                y = i * CELL_SIZE
                rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
                
                # Determine cell color
                pos = (i, j)
                
                if self.grid.is_wall(pos):
                    color = COLORS['wall']
                elif self.grid.is_start(pos):
                    color = COLORS['start']
                elif self.grid.is_target(pos):
                    color = COLORS['target']
                elif pos in self.grid.path:
                    color = COLORS['path']
                elif pos in self.grid.explored:
                    color = COLORS['explored']
                elif pos in self.grid.frontier:
                    color = COLORS['frontier']
                else:
                    color = COLORS['empty']
                
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, COLORS['grid_line'], rect, 1)
                
                # Draw coordinates
                if not self.grid.is_wall(pos):
                    text = self.small_font.render(f"{i},{j}", True, (100, 100, 100))
                    self.screen.blit(text, (x + 5, y + 5))
    
    def draw_info_panel(self):
        """Draw information panel at bottom"""
        panel_y = self.grid.size * CELL_SIZE
        panel_rect = pygame.Rect(0, panel_y, WINDOW_WIDTH, 80)
        pygame.draw.rect(self.screen, COLORS['panel'], panel_rect)
        pygame.draw.line(self.screen, COLORS['grid_line'], 
                        (0, panel_y), (WINDOW_WIDTH, panel_y), 2)
        
        # Current algorithm
        algo_name = self.algorithms[self.current_algo][0]
        algo_text = f"Algorithm: {algo_name}"
        text = self.font.render(algo_text, True, COLORS['text'])
        self.screen.blit(text, (10, panel_y + 5))
        
        # Status
        status = "SEARCHING..." if self.searching else "READY"
        status_color = (0, 150, 0) if self.searching else (150, 0, 0)
        status_text = f"Status: {status}"
        text = self.font.render(status_text, True, status_color)
        self.screen.blit(text, (10, panel_y + 30))
        
        # Instructions
        inst_text = "SPACE: Start | R: Reset | ←/→: Change Algorithm"
        text = self.small_font.render(inst_text, True, COLORS['text'])
        self.screen.blit(text, (10, panel_y + 55))
        
        # Statistics
        stats_text = f"Explored: {len(self.grid.explored)} | Frontier: {len(self.grid.frontier)}"
        text = self.small_font.render(stats_text, True, COLORS['text'])
        self.screen.blit(text, (WINDOW_WIDTH - 250, panel_y + 30))
    
    def handle_events(self):
        """Handle user input"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not self.searching:
                    # Start search
                    print(f"\n🚀 Starting {self.algorithms[self.current_algo][0]} search...")
                    self.grid.reset_search()
                    algo_func = self.algorithms[self.current_algo][1]
                    self.search_generator = algo_func(self.grid)
                    self.searching = True
                
                elif event.key == pygame.K_r:
                    # Reset grid
                    self.grid = Grid(GRID_SIZE)
                    self.searching = False
                    self.search_generator = None
                    print("\n🔄 Grid reset")
                
                elif event.key == pygame.K_LEFT:
                    self.current_algo = (self.current_algo - 1) % len(self.algorithms)
                    print(f"\n📌 Selected: {self.algorithms[self.current_algo][0]}")
                
                elif event.key == pygame.K_RIGHT:
                    self.current_algo = (self.current_algo + 1) % len(self.algorithms)
                    print(f"\n📌 Selected: {self.algorithms[self.current_algo][0]}")
        
        return True
    
    def run(self):
        """Main game loop"""
        running = True
        
        while running:
            running = self.handle_events()
            
            # Run search step
            if self.searching and self.search_generator:
                try:
                    next(self.search_generator)
                except StopIteration:
                    self.searching = False
                    if not self.grid.path:
                        print("❌ No path found!")
            
            # Draw everything
            self.screen.fill(COLORS['empty'])
            self.draw_grid()
            self.draw_info_panel()
            pygame.display.flip()
            
            # Control speed
            self.clock.tick(FPS)
        
        pygame.quit()
        sys.exit()


# ==================== MAIN ====================
def main():
    """Main entry point"""
    print("=" * 50)
    print("AI 2002 - Assignment 1 (Question 7)")
    print("Uninformed Search Algorithms in Grid Environment")
    print("=" * 50)
    print("\nControls:")
    print("  SPACE - Start search")
    print("  R - Reset grid")
    print("  ←/→ - Change algorithm")
    print("\nAlgorithms: BFS, DFS, UCS, DLS, IDDFS, Bidirectional")
    print("=" * 50)
    
    gui = GUI()
    gui.run()


if __name__ == "__main__":
    main()
import heapq
from cell import Cell
import time
import random


class Maze:
    def __init__(
            self,
            x1,
            y1,
            num_rows,
            num_cols,
            cell_size_x,
            cell_size_y,
            win=None,
            seed=None
    ):
        self._cells = []
        self._x1 = x1
        self._y1 = y1
        self._num_rows = num_rows
        self._num_cols = num_cols
        self._cell_size_x = cell_size_x
        self._cell_size_y = cell_size_y
        self._win = win
        if seed:
            random.seed(seed)

        self._create_cells()
        self._break_entrance_and_exit()
        self._break_walls_r(0, 0)
        self._reset_cells_visited()

    def _create_cells(self):
        for i in range(self._num_cols):
            col_cells = []
            for j in range(self._num_rows):
                col_cells.append(Cell(self._win))
            self._cells.append(col_cells)
        for i in range(self._num_cols):
            for j in range(self._num_rows):
                self._draw_cell(i, j)

    def _draw_cell(self, i, j):
        if self._win is None:
            return
        x1 = self._x1 + i * self._cell_size_x
        y1 = self._y1 + j * self._cell_size_y
        x2 = x1 + self._cell_size_x
        y2 = y1 + self._cell_size_y
        self._cells[i][j].draw(x1, y1, x2, y2)
        self._animate()

    def _animate(self):
        if self._win is None:
            return
        self._win.redraw()
        time.sleep(0.05)

    def _break_entrance_and_exit(self):
        self._cells[0][0].has_top_wall = False
        self._draw_cell(0, 0)
        self._cells[-1][-1].has_bottom_wall = False
        self._draw_cell(self._num_cols - 1, self._num_rows - 1)

    def _break_walls_r(self, i, j):
        self._cells[i][j].visited = True
        while True:
            next_index_list = []

            if i > 0 and not self._cells[i - 1][j].visited:
                next_index_list.append((i - 1, j))
            if i < self._num_cols - 1 and not self._cells[i + 1][j].visited:
                next_index_list.append((i + 1, j))
            if j > 0 and not self._cells[i][j - 1].visited:
                next_index_list.append((i, j - 1))
            if j < self._num_rows - 1 and not self._cells[i][j + 1].visited:
                next_index_list.append((i, j + 1))

            if len(next_index_list) == 0:
                self._draw_cell(i, j)
                return

            direction_index = random.randrange(len(next_index_list))
            next_index = next_index_list[direction_index]

            if next_index[0] == i + 1:
                self._cells[i][j].has_right_wall = False
                self._cells[i + 1][j].has_left_wall = False
            if next_index[0] == i - 1:
                self._cells[i][j].has_left_wall = False
                self._cells[i - 1][j].has_right_wall = True
            if next_index[1] == j + 1:
                self._cells[i][j].has_bottom_wall = False
                self._cells[i][j + 1].has_top_wall = False
            if next_index[1] == j - 1:
                self._cells[i][j].has_top_wall = False
                self._cells[i][j - 1].has_bottom_wall = False

            self._break_walls_r(next_index[0], next_index[1])

    def _reset_cells_visited(self):
        for col in self._cells:
            for cell in col:
                cell.visited = False

    def _solve_dfs(self, i, j):
        self._animate()

        self._cells[i][j].visited = True

        if i == self._num_cols - 1 and j == self._num_rows - 1:
            return True

        if (
            i > 0
            and not self._cells[i][j].has_left_wall
            and not self._cells[i - 1][j].visited
        ):
            self._cells[i][j].draw_move(self._cells[i - 1][j])
            if self._solve_dfs(i - 1, j):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i - 1][j], True)

        if (
            i < self._num_cols - 1
            and not self._cells[i][j].has_right_wall
            and not self._cells[i + 1][j].visited
        ):
            self._cells[i][j].draw_move(self._cells[i + 1][j])
            if self._solve_dfs(i + 1, j):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i + 1][j], True)

        if (
            j > 0
            and not self._cells[i][j].has_top_wall
            and not self._cells[i][j - 1].visited
        ):
            self._cells[i][j].draw_move(self._cells[i][j - 1])
            if self._solve_dfs(i, j - 1):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i][j - 1], True)

        if (
            j < self._num_rows - 1
            and not self._cells[i][j].has_bottom_wall
            and not self._cells[i][j + 1].visited
        ):
            self._cells[i][j].draw_move(self._cells[i][j + 1])
            if self._solve_dfs(i, j + 1):
                return True
            else:
                self._cells[i][j].draw_move(self._cells[i][j + 1], True)

        return False
    
    def _reconstruct_path(self, came_from, start, goal):
        current = goal
        while current != start:
            self._animate()
            self._cells[current[0]][current[1]].draw_move(self._cells[came_from[current][0]][came_from[current][1]])
            current = came_from[current]
        return True
    
    def _solve_bfs(self, i, j):
        goal = (self._num_cols - 1, self._num_rows - 1)
        start = (i, j)

        cells_to_visit = [start]
        came_from = {start: None}

        while cells_to_visit:
            i, j = cells_to_visit.pop(0)

            if start == goal:
                return self._reconstruct_path(came_from, start, goal)
            
            self._animate()
            self._cells[i][j].visited = True

            if (
                i > 0
                and not self._cells[i][j].has_left_wall
                and not self._cells[i - 1][j].visited
            ):
                cells_to_visit.append((i - 1, j))
                came_from[(i - 1, j)] = (i, j)
                self._cells[i][j].draw_move(self._cells[i - 1][j],True)

            if (
                i < self._num_cols - 1
                and not self._cells[i][j].has_right_wall
                and not self._cells[i + 1][j].visited
            ):
                cells_to_visit.append((i + 1, j))
                came_from[(i + 1, j)] = (i, j)
                self._cells[i][j].draw_move(self._cells[i + 1][j], True)

            if (
                j > 0
                and not self._cells[i][j].has_top_wall
                and not self._cells[i][j - 1].visited
            ):
                cells_to_visit.append((i, j - 1))
                came_from[(i, j - 1)] = (i, j)
                self._cells[i][j].draw_move(self._cells[i][j - 1], True)

            if (
                j < self._num_rows - 1
                and not self._cells[i][j].has_bottom_wall
                and not self._cells[i][j + 1].visited
            ):
                cells_to_visit.append((i, j + 1))
                came_from[(i, j + 1)] = (i, j)
                self._cells[i][j].draw_move(self._cells[i][j + 1], True)

        return False
    
    def _a_star(self, i, j):
        def h(current, goal):
            return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

        goal = (self._num_cols - 1, self._num_rows - 1)
        start = (i, j)

        openSet = []
        heapq.heappush(openSet, (h(start, goal), start))

        came_from = {}
        gScore = {start: 0}
        fScore = {start: h(start, goal)}

        while openSet:
            self._animate()
            _, current = heapq.heappop(openSet)

            if current == goal:
                return self._reconstruct_path(came_from, start, goal)

            current_i, current_j = current
            self._cells[current_i][current_j].visited = True

            neighbors = [
                (current_i - 1, current_j), 
                (current_i + 1, current_j), 
                (current_i, current_j - 1), 
                (current_i, current_j + 1)
            ]

            for neighbor in neighbors:
                ni, nj = neighbor

                if 0 <= ni < self._num_cols and 0 <= nj < self._num_rows:
                    if self._cells[current_i][current_j].has_left_wall and ni == current_i - 1:
                        continue
                    if self._cells[current_i][current_j].has_right_wall and ni == current_i + 1:
                        continue
                    if self._cells[current_i][current_j].has_top_wall and nj == current_j - 1:
                        continue
                    if self._cells[current_i][current_j].has_bottom_wall and nj == current_j + 1:
                        continue

                    tentative_gScore = gScore[current] + 1

                    if tentative_gScore < gScore.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        gScore[neighbor] = tentative_gScore
                        fScore[neighbor] = tentative_gScore + h(neighbor, goal)

                        if neighbor not in [item[1] for item in openSet]:
                            heapq.heappush(openSet, (fScore[neighbor], neighbor))
                            self._cells[current_i][current_j].draw_move(self._cells[ni][nj], True)

        return False



    def solve(self):
        # return self._solve_dfs(0, 0)
        # return self._solve_bfs(0, 0)
        return self._a_star(0, 0)

use crate::maze::{Maze, Cell};

pub fn solve(maze: &mut Maze) -> bool {
    let (start_x, start_y) = maze.start;
    let (end_x, end_y) = maze.end;

    let mut visited = vec![vec![false; maze.width]; maze.height];
    
    return depth_first_search(maze, start_x, start_y, end_x, end_y, &mut visited);
}

fn depth_first_search(
    maze: &mut Maze,
    x: usize,
    y: usize,
    end_x: usize,
    end_y: usize,
    visited: &mut Vec<Vec<bool>>,
) -> bool {
    if x == end_x && y == end_y {
        return true; // Found the end
    }

    if visited[y][x] || maze.grid[y][x] == Cell::Wall {
        return false;
    }

    visited[y][x] = true;

    // Mark the current cell as part of the solution path (this is for visualization)
    maze.grid[y][x] = Cell::Solution;

    // Explore neighbors (up, down, left, right)
    let directions = &[(0, 1), (1, 0), (0, -1), (-1, 0)];

    for &(dx, dy) in directions.iter() {
        let nx = (x as isize + dx) as usize;
        let ny = (y as isize + dy) as usize;

        // Ensure we are within bounds of the maze
        if nx < maze.width && ny < maze.height {
            if depth_first_search(maze, nx, ny, end_x, end_y, visited) {
                return true; // Solution found
            }
        }
    }

    // Unmark the current cell if it is not part of the solution path
    maze.grid[y][x] = Cell::Path;
    false
}
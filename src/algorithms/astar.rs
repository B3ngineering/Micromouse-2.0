use crate::maze::{Maze, Cell};
use std::collections::BinaryHeap;
use std::cmp::Ordering;

#[derive(Copy, Clone, Eq, PartialEq)]
struct Node {
    position: (usize, usize),
    f_score: usize,
}

// Implement ordering for the priority queue (min-heap)
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.cmp(&self.f_score) // Reversed to create a min-heap
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn solve(maze: &mut Maze) -> bool {
    let (start_x, start_y) = maze.start;
    let (end_x, end_y) = maze.end;

    // Priority queue for exploring nodes (min-heap based on F-score)
    let mut open_set = BinaryHeap::new();
    open_set.push(Node {
        position: (start_x, start_y),
        f_score: heuristic(start_x, start_y, end_x, end_y),
    });

    // G-score: cost from start to the current node
    let mut g_score = vec![vec![usize::MAX; maze.width]; maze.height];
    g_score[start_y][start_x] = 0;

    // Parent map for reconstructing the path
    let mut came_from = vec![vec![None; maze.width]; maze.height];

    // Directions to explore (up, down, left, right)
    let directions = [(0, 1), (1, 0), (0, -1), (-1, 0)];

    while let Some(Node { position: (x, y), .. }) = open_set.pop() {
        // Check if we've reached the end
        if (x, y) == (end_x, end_y) {
            reconstruct_path(maze, came_from, x, y);
            return true;
        }

        // Explore neighbors
        for &(dx, dy) in directions.iter() {
            let nx = (x as isize + dx) as usize;
            let ny = (y as isize + dy) as usize;

            // Skip if out of bounds or if it's a wall
            if nx >= maze.width || ny >= maze.height || maze.grid[ny][nx] == Cell::Wall {
                continue;
            }

            // Tentative G-score (cost to reach neighbor)
            let tentative_g_score = g_score[y][x] + 1;

            // Only continue if we found a cheaper way to the neighbor
            if tentative_g_score < g_score[ny][nx] {
                came_from[ny][nx] = Some((x, y));
                g_score[ny][nx] = tentative_g_score;

                // F-score = G + H
                let f_score = tentative_g_score + heuristic(nx, ny, end_x, end_y);

                open_set.push(Node {
                    position: (nx, ny),
                    f_score,
                });
            }
        }
    }

    // If we exhaust the open set, no path was found
    false
}

// Heuristic function (Manhattan distance)
fn heuristic(x: usize, y: usize, end_x: usize, end_y: usize) -> usize {
    ((x as isize - end_x as isize).abs() + (y as isize - end_y as isize).abs()) as usize
}

// Reconstruct the path from the end to the start
fn reconstruct_path(maze: &mut Maze, came_from: Vec<Vec<Option<(usize, usize)>>>, mut x: usize, mut y: usize) {
    while let Some((prev_x, prev_y)) = came_from[y][x] {
        // Mark the solution path
        maze.grid[y][x] = Cell::Solution;
        x = prev_x;
        y = prev_y;
    }
    // Mark the start as part of the solution path
    maze.grid[y][x] = Cell::Solution;
}
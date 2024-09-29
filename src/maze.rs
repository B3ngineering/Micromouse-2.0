use rand::prelude::SliceRandom;
use std::fmt;

#[derive(Clone, Copy, PartialEq)]
pub enum Cell {
    Wall,
    Path,
    Start,
    End,
    Solution,
}

pub struct Maze {
    pub width: usize,
    pub height: usize,
    pub grid: Vec<Vec<Cell>>,
    pub start: (usize, usize),
    pub end: (usize, usize),
}

impl Maze {
    pub fn new(width: usize, height: usize) -> Self {
        let grid = vec![vec![Cell::Wall; width]; height];
        Maze {
            width,
            height,
            grid,
            start: (1, 1),
            end: (width - 2, height - 2),
        }
    }

    pub fn generate(&mut self) {
        // Start maze generation at (1, 1)
        self.carve_passage(1, 1);
        self.set_start_end();
    }

    fn carve_passage(&mut self, x: usize, y: usize) {
        let directions = &[(0, 2), (2, 0), (0, -2), (-2, 0)];
        let mut rng = rand::thread_rng();
        let mut directions_shuffled = directions.clone();
        directions_shuffled.shuffle(&mut rng);

        // Mark current cell as part of the path
        self.grid[y][x] = Cell::Path;

        for &(dx, dy) in directions_shuffled.iter() {
            let nx = (x as isize + dx) as usize;
            let ny = (y as isize + dy) as usize;

            // Check if the cell is within bounds and is a wall
            if nx > 0 && nx < self.width - 1 && ny > 0 && ny < self.height - 1 && self.grid[ny][nx] == Cell::Wall {
                // Remove the wall between the current cell and the next cell
                self.grid[(y as isize + dy / 2) as usize][(x as isize + dx / 2) as usize] = Cell::Path;
                // Recursively carve passages from the next cell
                self.carve_passage(nx, ny);
            }
        }
    }

    // Set the start and end points of the maze
    fn set_start_end(&mut self) {
        self.grid[self.start.1][self.start.0] = Cell::Start;
        self.grid[self.end.1][self.end.0] = Cell::End;
    }
}

impl fmt::Display for Maze {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for row in &self.grid {
            for &cell in row {
                let symbol = match cell {
                    Cell::Wall => "#",
                    Cell::Path => " ",
                    Cell::Start => "S", // Start point
                    Cell::End => "E",   // End point
                    Cell::Solution => ".",
                };
                write!(f, "{}", symbol)?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}
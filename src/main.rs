mod maze;
mod algorithms;
use crate::maze::Maze;


fn main() {
    let width = 33;
    let height = 33;

    let mut maze = Maze::new(width, height);
    maze.generate();

    println!("{}", maze);
    if algorithms::astar::solve(&mut maze) {
        println!("Solved Maze with A-Star:");
        println!("{}", maze);
    } else {
        println!("No solution found!");
    }
    if algorithms::dfs::solve(&mut maze) {
        println!("Solved Maze with DFS:");
        println!("{}", maze);
    } else {
        println!("No solution found!");
    }
}

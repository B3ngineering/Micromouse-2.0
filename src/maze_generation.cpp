#include <iostream>
#include <vector>
#include <stack>
#include <cstdlib>
#include <ctime>

#include "maze_world.h"

using namespace std;

class MazeGenerator {

public:
    int width, height;
    vector<vector<int>> maze;

    MazeGenerator(int width, int height) 
        : width(width), height(height), maze(2 * width + 1, vector<int>(2 * height + 1, 0)) {
        srand(time(nullptr));
        }
    
    // Generate the maze using recursive backtracking algorithm
    void generateMaze() {

        // Generate a random start point on the stack
        stack<pair<int, int>> stack;
        int startX = rand() % width;
        int startY = rand() % height;
        stack.push({startX, startY});

        while(!stack.empty()) {

            // Mark the cell in the stack as visited
            auto [x, y] = stack.top();
            maze[2 * x + 1][2 * y + 1] = 1;

            // Find all neighbours of the current cell
            vector<pair<int, int>> neighbours;
            if (x > 0 && maze[2 * x - 1][2 * y + 1] == 0) neighbours.push_back({x - 1, y});
            if (x < width - 1 && maze[2 * x + 3][2 * y + 1] == 0) neighbours.push_back({x + 1, y});
            if (y > 0 && maze[2 * x + 1][2 * y - 1] == 0) neighbours.push_back({x, y - 1});
            if (y < height - 1 && maze[2 * x + 1][2 * y + 3] == 0) neighbours.push_back({x, y + 1});

            // Knock down walls to form path
            if (neighbours.empty()) {
                stack.pop();
            } else {
                auto next = neighbours[rand() % neighbours.size()];
                if (next.first > x) maze[2 * x + 2][2 * y + 1] = 1;
                if (next.first < x) maze[2 * x][2 * y + 1] = 1;
                if (next.second > y) maze[2 * x + 1][2 * y + 2] = 1;
                if (next.second < y) maze[2 * x + 1][2 * y] = 1;
                stack.push(next);
            }
        }
    }

    void printMaze() {
        for (const auto& row : maze) {
            for (int cell : row) {
                cout << (cell ? "  " : "██");
            }
            cout << endl;
        }
    }
};


int main() {
    MazeGenerator my_maze(10, 10);
    my_maze.generateMaze();
    my_maze.printMaze();

    pair<int, int> start, end;

    // Choose random start and end points
    start = {1, 1};
    end = {19, 19};

    generateMazeWorld(my_maze.maze, "maze.world", start, end);
    return 0;
}
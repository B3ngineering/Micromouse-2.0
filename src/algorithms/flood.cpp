#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>
#include <climits>

#include "algorithms.h"

using namespace std;

vector<pair<int, int>> flood_fill(vector<vector<int>>& maze, pair<int, int> start, pair<int, int> goal) {
    // Setup for flood fill algorithm
    const int MAZE_SIZE = maze.size();
    const vector<pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    queue<pair<int, int>> q;
    vector<vector<int>> distance(MAZE_SIZE, vector<int>(MAZE_SIZE, INT_MAX));
    vector<vector<pair<int, int>>> parent(MAZE_SIZE, vector<pair<int, int>>(MAZE_SIZE, {-1, -1}));

    q.push(goal);
    distance[goal.first][goal.second] = 0;

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE && maze[nx][ny] == 0 && distance[nx][ny] == INT_MAX) {
                distance[nx][ny] = distance[x][y] + 1;
                parent[nx][ny] = {x, y};
                q.push({nx, ny});
            }
        }
    }

    // Trace the shortest path from start to goal
    vector<pair<int, int>> path;
    if (distance[start.first][start.second] != INT_MAX) {
        pair<int, int> current = start;
        while (current != goal) {
            path.push_back(current);
            current = parent[current.first][current.second];
        }
        path.push_back(goal);
    }

    return path;
}

int main() {
     // Read the maze from the binary file
    vector<vector<int>> maze = readMaze("maze/maze.bin");

    // Display the loaded maze with 0s and 1s swapped
    for (auto& row : maze) {
        for (int& cell : row) {
            cell = (cell == 0) ? 1 : 0;
            cout << cell << " ";
        }
        cout << "\n";
    }

    // Define start and goal coordinates
    pair<int, int> start = {1, 1};
    pair<int, int> goal = {19, 19};

    vector<pair<int, int>> path = flood_fill(maze, start, goal);

    // Display the path
    for (auto [x, y] : path) {
        cout << "(" << x << ", " << y << ") -> ";
    }
    cout << "\n";

    // Save the path to a binary file
    ofstream outFile("../ros2_ws/src/micromouse_description/src/path.bin", ios::binary);
    if (outFile.is_open()) {
        int pathSize = path.size();
        outFile.write(reinterpret_cast<char*>(&pathSize), sizeof(pathSize));
        for (const auto& [x, y] : path) {
            outFile.write(reinterpret_cast<const char*>(&x), sizeof(x));
            outFile.write(reinterpret_cast<const char*>(&y), sizeof(y));
        }
        outFile.close();
    } else {
        cerr << "Unable to open file for writing\n";
    }
    return 0;
}
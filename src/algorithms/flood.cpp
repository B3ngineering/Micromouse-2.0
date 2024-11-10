#include <iostream>
#include <fstream>
#include <vector>

#include "algorithms.h"

using namespace std;

int main() {
     // Read the maze from the binary file
    vector<vector<int>> maze = readMaze("maze.bin");

    // Display the loaded maze with 0s and 1s swapped
    for (auto& row : maze) {
        for (int& cell : row) {
            cell = (cell == 0) ? 1 : 0;
            cout << cell << " ";
        }
        cout << "\n";
    }

    return 0;
}
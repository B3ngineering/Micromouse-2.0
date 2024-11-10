#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

vector<vector<int>> readMaze(const string& filename) {
    cout << "Opening file: " << filename << endl;
    ifstream inFile(filename, ios::binary);
    if (!inFile) {
        cerr << "Could not open the file!" << endl;
        return {};
    }

    // Read dimensions
    int rows, cols;
    inFile.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    inFile.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    cout << "Maze dimensions: " << rows << "x" << cols << endl;

    // Prepare the maze container
    vector<vector<int>> maze(rows, vector<int>(cols));

    // Read the maze data
    for (int i = 0; i < rows; ++i) {
        inFile.read(reinterpret_cast<char*>(maze[i].data()), maze[i].size() * sizeof(int));
        cout << "Read row " << i << ": ";
        for (int cell : maze[i]) {
            cout << cell << " ";
        }
        cout << endl;
    }
    inFile.close();
    cout << "Finished reading the maze." << endl;

    return maze;
}
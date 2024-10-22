#include <fstream>

using namespace std;

void generateMazeWorld(const vector<vector<int>>& maze, const string& filename) {
    ofstream worldFile(filename);

    if (!worldFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return;
    }

    // General world settings
    worldFile << "sdf version='1.7'>\n";
    worldFile << "<world name=\"maze_world\">\n";
    worldFile << "  <include>\n";
    worldFile << "    <uri>model://ground_plane</uri>\n";
    worldFile << "  </include>\n";
    worldFile << "  <include>\n";
    worldFile << "    <uri>model://sun</uri>\n";
    worldFile << "  </include>\n";

    // Generate maze walls
    for (int y = 0; y < maze.size(); y++) {
        for (int x = 0; x < maze[0].size(); x++) {
            if (maze[y][x] == 0) {
                worldFile << "  <model name=\"wall_" << x << "_" << y << "\">\n";
                worldFile << "    <pose>" << x << " " << y << " 0 0 0 0</pose>\n";
                worldFile << "    <link name=\"link\">\n";
                worldFile << "      <collision name=\"collision\">\n";
                worldFile << "        <geometry>\n";
                worldFile << "          <box>\n";
                worldFile << "            <size>1 1 1</size>\n";
                worldFile << "          </box>\n";
                worldFile << "        </geometry>\n";
                worldFile << "      </collision>\n";
                worldFile << "      <visual name=\"visual\">\n";
                worldFile << "        <geometry>\n";
                worldFile << "          <box>\n";
                worldFile << "            <size>1 1 1</size>\n";
                worldFile << "          </box>\n";
                worldFile << "        </geometry>\n";
                worldFile << "      </visual>\n";
                worldFile << "    </link>\n";
                worldFile << "  </model>\n";
            }
        }
    }

    worldFile << "</world>\n";
    worlfFile << "</sdf>\n";
    worldFile.close();
}
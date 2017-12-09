#pragma once

#include <sstream>
#include <fstream>
#include <vector>

namespace particle_filter_project {

namespace tools {

using std::string;
using std::move;
using std::cerr;
using std::endl;
using std::ifstream;
using std::istringstream;

template <typename T>
void CheckFile(const T& file, const char* name) {
  if (not file.is_open()) {
    cerr << "Cannot open input file: " << name << endl;
    exit(EXIT_FAILURE);
  }
}

/* Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
void ReadMapData(string filename, Map& map) {
  ifstream map_file{filename.c_str(), ifstream::in};
  tools::CheckFile(map_file, filename.c_str());
  
  string line{};
  istringstream iss{};

  while(getline(map_file, line)) {
    iss.str(line);
    Landmark lmark{};
    iss >> lmark;

    map.push_back(move(lmark));
    iss.clear();
  }
}

} // namespace tools

} // namespace particle_filter_project 

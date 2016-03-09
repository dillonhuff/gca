#include <cassert>
#include <dirent.h>
#include <iostream>

#include "core/arena_allocator.h"

using namespace gca;
using namespace std;

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void read_dir(const string& dir_name) {
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string fname = ent->d_name;
      if (fname != "." && fname != "..") {
	read_dir(dir_name + "/" + fname);
	//cout << "Name: " << fname << endl;
      }
    }
    closedir(dir);
  } else {
    if (ends_with(dir_name, ".NCF")) {
      cout << dir_name << endl;
      //	print_knife_hacks(read_file(dir_name + fname));
    }
    //cout << "Could not open directory: " << dir_name << endl;
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-knife-hacks <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];
  read_dir(dir_name);
}

#include <cassert>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "core/arena_allocator.h"
#include "core/lexer.h"
#include "core/parser.h"

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
      }
    }
    closedir(dir);
  } else {
    if (ends_with(dir_name, ".NCF")) {
      cout << dir_name << endl;
      std::ifstream t(dir_name);
      std::string str((std::istreambuf_iterator<char>(t)),
		      std::istreambuf_iterator<char>());
      vector<token> p = lex_gprog(str);
      //gprog* p = parse_gprog(str);
      cout << "NUM INSTRUCTIONS: " << p.size() << endl;
    }
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
  // gprog* p = read_file("/Users/dillon/Documents/MattNorcia318/Gcode/Bottom.NCF");
  // cout << *p << endl;
  read_dir(dir_name);
}

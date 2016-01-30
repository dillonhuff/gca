#include <cassert>
#include <dirent.h>
#include <iostream>

#include "analysis/extract_cuts.h"
#include "core/basic_states.h"
#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"
#include "synthesis/align_blade.h"

using namespace gca;
using namespace std;

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

unsigned hack_length = 8;

bool is_align_hack(const vector<instr*>& window) {
  if (window.size() != hack_length) { return false; }
  if (!window[0]->is_G1()) { return false; }
  if (!window[1]->is_G1()) { return false; }
  if (!window[2]->is_G0()) { return false; }
  if (!window[3]->is_G0()) { return false; }
  if (!window[4]->is_G1()) { return false; }
  if (!(window[5]->is_G2() || window[3]->is_G3())) { return false; }
  if (!window[6]->is_G1()) { return false; }
  if (!window[7]->is_G1()) { return false; }
  return true;
}

void print_knife_hacks(gprog* prog) {
  vector<instr*> window;
  for (unsigned i = 0; i < prog->size(); i++) {
    instr* is = (*prog)[i];
    if (window.size() >= hack_length) {
      window.erase(window.begin());
    }
    window.push_back(is);
    if (is_align_hack(window)) {
      cout << endl << "-- window " << endl;
      for (unsigned j = 0; j < window.size(); j++) {
	cout << *(window[j]) << endl;
      }
    }
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: extract-g1-paths <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string fname = ent->d_name;
      if (ends_with(fname, ".tap")) {
	print_knife_hacks(read_file(dir_name + fname));
      }
    }
    closedir(dir);
  } else {
    cout << "Could not open directory: " << dir_name << endl;
    return EXIT_FAILURE;
  }
}

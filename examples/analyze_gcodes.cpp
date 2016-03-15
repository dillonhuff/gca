#include <cassert>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "analysis/machine_state.h"
#include "analysis/unfold.h"
#include "core/arena_allocator.h"
#include "core/lexer.h"
#include "core/parser.h"

using namespace gca;
using namespace std;

void print_climb_vs_conventional(const machine_state& s) {
  if (s.tool_radius_comp == TOOL_RADIUS_COMP_LEFT) {
    if (s.spindle_setting == SPINDLE_OFF ||
	s.spindle_setting == SPINDLE_STATE_UNKNOWN) {
      cout << "Bad spindle state: " << endl;
      cout << s << endl;
      assert(false);
    }
    if (s.spindle_setting == SPINDLE_CLOCKWISE) {
      cout << "Climb" << endl;
    } else if (s.spindle_setting == SPINDLE_COUNTERCLOCKWISE) {
      cout << "Conventional" << endl;
    }
  } else if (s.tool_radius_comp == TOOL_RADIUS_COMP_RIGHT) {
    if (s.spindle_setting == SPINDLE_OFF ||
	s.spindle_setting == SPINDLE_STATE_UNKNOWN) {
      cout << "Bad spindle state: " << endl;
      cout << s << endl;
      assert(false);
    }
    if (s.spindle_setting == SPINDLE_COUNTERCLOCKWISE) {
      cout << "Climb" << endl;
    } else if (s.spindle_setting == SPINDLE_CLOCKWISE) {
      cout << "Conventional" << endl;
    }
  }
}

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

bool is_cut(const machine_state& s) {
  return (s.active_move_type != FAST_MOVE) && !(s.x->is_omitted() || s.y->is_omitted() || s.z->is_omitted());
}

bool spindle_off(const machine_state& s) {
  return (s.spindle_setting == SPINDLE_OFF ||
	  s.spindle_setting == SPINDLE_STATE_UNKNOWN);
}

void sanity_check_machine_state(const machine_state& s) {
  if (is_cut(s)) {
    if (spindle_off(s)) {
      cout << "Spindle off during cut: " << endl;
      cout << s << endl;
      assert(false);
    }
  }
}

void print_program_info(const string& dir_name) {
  if (ends_with(dir_name, ".NCF")) {
    cout << dir_name << endl;
    std::ifstream t(dir_name);
    std::string str((std::istreambuf_iterator<char>(t)),
		    std::istreambuf_iterator<char>());
    vector<block> p = lex_gprog(str);
    cout << "NUM BLOCKS: " << p.size() << endl;
    vector<machine_state> states = all_program_states(p);
    cout << "STATES: " << states.size() << endl;
    //for_each(states.begin(), states.end(), sanity_check_machine_state);
    for_each(states.begin(), states.end(), print_climb_vs_conventional);
  }
}

template<typename T>
void read_dir(const string& dir_name, T f) {
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string fname = ent->d_name;
      if (fname != "." && fname != "..") {
	read_dir(dir_name + "/" + fname, f);
      }
    }
    closedir(dir);
  } else {
    f(dir_name);
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
  time_t start;
  time_t end;
  time(&start);
  read_dir(dir_name, print_program_info);
  time(&end);
  double seconds = difftime(end, start);
  cout << "Total time to process all .NCF files: " << seconds << endl;
}
// Before unfold and create program states were collapsed into one: 128 seconds
// After: 123

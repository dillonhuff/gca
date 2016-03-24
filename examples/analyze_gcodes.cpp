#include <cassert>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/unfold.h"
#include "analysis/utils.h"
#include "core/arena_allocator.h"
#include "core/lexer.h"
#include "geometry/box.h"
#include "system/algorithm.h"

using namespace gca;
using namespace std;

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
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

template<typename F>
void apply_to_gprograms(const string& dn, F f) {
  auto func = [&f](const string& dir_name) {
    if (ends_with(dir_name, ".NCF")) {
      cout << dir_name << endl;
      std::ifstream t(dir_name);
      std::string str((std::istreambuf_iterator<char>(t)),
		      std::istreambuf_iterator<char>());
      vector<block> p = lex_gprog(str);
      cout << "NUM BLOCKS: " << p.size() << endl;
      vector<machine_state> states = all_program_states(p);
      cout << "STATES: " << states.size() << endl;
      f(states);
    }
  };
  read_dir(dn, func);
}

void split_and_print(const vector<machine_state>& toolpath) {
  auto ptbl = select_column(G54_COORD_SYSTEM, program_position_table(toolpath));
  vector<pair<machine_state, position>> tstates;
  for (unsigned i = 0; i < toolpath.size(); i++) {
    tstates.push_back({toolpath[i], ptbl[i]});
  }
  vector<vector<pair<machine_state, position>>> sub_paths;
  split_by(tstates, sub_paths,
  	   [](const pair<machine_state, position>& x,
  	      const pair<machine_state, position>& y)
  	   { return x.second.is_lit() == y.second.is_lit(); });
  delete_if(sub_paths, [](const vector<pair<machine_state, position>>& p)
  	    { return !p.front().second.is_lit(); });
  for (auto path : sub_paths) {
    assert(all_of(path.begin(), path.end(),
    		  [](const pair<machine_state, position>& p)
    		  { return p.second.is_lit(); }));
    vector<machine_state> states;
    
  }
}

void print_toolpaths(const vector<machine_state>& states) {
  vector<vector<machine_state>> toolpaths;
  split_by(states, toolpaths, [](const machine_state& c, const machine_state& p)
	   { return c.active_tool == p.active_tool; });
  delete_if(toolpaths, [](const vector<machine_state>& c)
	    { return c.back().active_tool->is_omitted(); });
  cout << "Number of toolpaths with known tool: " << toolpaths.size() << endl;
  // On HAAS it appears that the last toolpath is always a change back
  // to the first tool that was active, this toolpath never contains
  // any moves
  if (toolpaths.size() > 0) { toolpaths.pop_back(); };
  for (auto toolpath : toolpaths) {
    if (is_analyzable(toolpath)) {
      split_and_print(toolpath);
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
  time_t start;
  time_t end;
  time(&start);
  apply_to_gprograms(dir_name, [](const vector<machine_state>& p)
		     { print_toolpaths(p); });
  time(&end);
  double seconds = difftime(end, start);
  cout << "Total time to process all .NCF files: " << seconds << endl;
}

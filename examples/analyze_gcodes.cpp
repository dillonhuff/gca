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
#include "core/parser.h"
#include "system/algorithm.h"

using namespace gca;
using namespace std;

void print_climb_vs_conventional(const machine_state& s) {
  if (s.tool_radius_comp == TOOL_RADIUS_COMP_LEFT) {
    if (s.spindle_setting == SPINDLE_OFF ||
	s.spindle_setting == SPINDLE_STATE_UNKNOWN) {
      cout << "Bad spindle state: " << s.line_no << endl;
      cout << s << endl;
      assert(false);
    }
    if (s.spindle_setting == SPINDLE_CLOCKWISE) {
    } else if (s.spindle_setting == SPINDLE_COUNTERCLOCKWISE) {
      cout << "Conventional at line " << s.line_no << endl;
    }
  } else if (s.tool_radius_comp == TOOL_RADIUS_COMP_RIGHT) {
    if (s.spindle_setting == SPINDLE_OFF ||
	s.spindle_setting == SPINDLE_STATE_UNKNOWN) {
      cout << "Bad spindle state: " << s.line_no << endl;
      cout << s << endl;
      assert(false);
    }
    if (s.spindle_setting == SPINDLE_COUNTERCLOCKWISE) {
    } else if (s.spindle_setting == SPINDLE_CLOCKWISE) {
      cout << "Conventional at line " << s.line_no << endl;
    }
  }
}

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
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

void print_toolpath_info(const vector<machine_state>& toolpath) {
  assert(toolpath.size() > 0);
  cout << toolpath << endl;
  cout << "Active tool: " << *(toolpath.back().active_tool) << endl;
  // TODO: Add support for any toolpath in the same coordinate system.
  // In the existing GCODE programs from the PRL, this does not seem
  // to come up
  if (all_of(toolpath.begin(), toolpath.end(),
	     [](const machine_state& s)
	     { return s.active_coord_system == G54_COORD_SYSTEM; })) {
    position_table t = program_position_table(toolpath);
    cout << "Position table: " << endl;
    cout << t << endl;
    assert(false);
    vector<position> positions = select_column(G54_COORD_SYSTEM, t);
    assert(positions.size() == t.size());
    vector<point> pts;
    for (auto p : positions) {
      cout << "p = " << p << endl;
      if (p.is_lit()) { pts.push_back(p.extract_point()); }
    }
    assert(pts.size() > 0);
    cout << "\t# Positions: " << pts.size() << endl;
    auto xminmax = minmax_element(pts.begin(), pts.end(),
				  [](const point l, const point r)
				  { return l.x < r.x; });
    assert(xminmax.first != pts.end());
    point p = *(xminmax.first);
    cout << "p = " << p << endl;
    cout << "Bounds " << endl;
    cout << "\tX min = " << *(xminmax.first) << endl;
    cout << "\tX max = " << *(xminmax.second) << endl;
  } else {
    cout << "Cannot analyze toolpath, not all moves are in G54" << endl;
  }
}

void analyze_toolpaths(const vector<machine_state>& states) {
  cout << "Analyzing toolpaths..." << endl;
  vector<vector<machine_state>> toolpaths;
  split_by(states, toolpaths, [](const machine_state& c, const machine_state& p)
	   { return c.active_tool == p.active_tool; });
  cout << "Number of toolpaths: " << toolpaths.size() << endl;
  delete_if(toolpaths, [](const vector<machine_state>& c)
	    { return c.back().active_tool->is_omitted(); });
  cout << "Number of toolpaths with known tool: " << toolpaths.size() << endl;
  // On HAAS it appears that the last toolpath is always a change back
  // to the first tool that was active, this toolpath never contains
  // any moves
  if (toolpaths.size() > 0) { toolpaths.pop_back(); };
  for (auto toolpath : toolpaths) {
    print_toolpath_info(toolpath);
  }
  cout << "Done analyzing" << endl;
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
    analyze_toolpaths(states);
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

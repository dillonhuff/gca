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

box bound_positions(const vector<point> pts) {
  assert(pts.size() > 0);
  auto xminmax = minmax_element(pts.begin(), pts.end(),
				[](const point l, const point r)
				{ return l.x < r.x; });
  assert(xminmax.first != pts.end());
  auto yminmax = minmax_element(pts.begin(), pts.end(),
				[](const point l, const point r)
				{ return l.y < r.y; });
  assert(yminmax.first != pts.end());
  return box((*(xminmax.first)).x, (*(xminmax.second)).x,
	     (*(yminmax.first)).y, (*(yminmax.second)).y);
}

bool is_analyzable(const vector<machine_state>& toolpath) {
  assert(toolpath.size() > 0);
  if (!all_of(toolpath.begin(), toolpath.end(),
	      [](const machine_state& s)
	      { return s.active_coord_system == G54_COORD_SYSTEM; })) {
    return false;
  }
  return true;
}

void print_toolpath_info(const vector<machine_state>& toolpath) {
  assert(toolpath.size() > 0);
  cout << "Active tool: " << *(toolpath.back().active_tool) << endl;
  if (all_of(toolpath.begin(), toolpath.end(),
	     [](const machine_state& s)
	     { return s.active_coord_system == G54_COORD_SYSTEM; })) {
    position_table t = program_position_table(toolpath);
    vector<position> positions = select_column(G54_COORD_SYSTEM, t);
    assert(positions.size() == t.size());
    vector<point> pts;
    for (auto p : positions) {
      if (p.is_lit()) { pts.push_back(p.extract_point()); }
    }
    box b = bound_positions(pts);
    cout << "Bounds: " << endl << b << endl;
  } else {
    cout << "Cannot analyze toolpath, not all moves are in G54" << endl;
  }
}

box toolpath_bounds(const vector<machine_state>& toolpath) {
    position_table t = program_position_table(toolpath);
    vector<position> positions = select_column(G54_COORD_SYSTEM, t);
    assert(positions.size() == t.size());
    vector<point> pts;
    for (auto p : positions) {
      if (p.is_lit()) { pts.push_back(p.extract_point()); }
    }
    return bound_positions(pts);
}

void analyze_toolpaths(const vector<machine_state>& states) {
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
  if (all_of(toolpaths.begin(), toolpaths.end(), is_analyzable)) {
    vector<box> bounding_boxes;
    for (auto t : toolpaths)
      { bounding_boxes.push_back(toolpath_bounds(t)); }
    for (unsigned i = 0; i < toolpaths.size(); i++) {
      box b1 = bounding_boxes[i];
      value* active_tool_1 = toolpaths[i].front().active_tool;
      for (unsigned j = i + 1; j < toolpaths.size(); j++) {
	box b2 = bounding_boxes[j];
	value* active_tool_2 = toolpaths[j].front().active_tool;
	if (*active_tool_1 == *active_tool_2) {
	  bool no_overlap = true;
	  for (unsigned k = i + 1; k < j; k++) {
	    if (overlap(bounding_boxes[k], b1) ||
		overlap(bounding_boxes[k], b2))
	      { no_overlap = false; }
	  }
	  if (no_overlap) {
	    cout << "Toolpaths " << i << " and " << j << " could be merged" << endl;
	    cout << "Toolpath " << i << " tool: " << endl << *active_tool_1 << endl;
	    cout << "Toolpath " << i << " bounds: " << endl << b1 << endl;
	    cout << "Toolpath " << j << " tool: " << endl << *active_tool_2 << endl;
	    cout << "Toolpath " << j << " bounds: " << endl << b2 << endl;
	    cout << "Toolpaths in between have bounds: " << endl;
	    for (unsigned k = i + 1; k < j; k++) {
	      cout << "K = " << k << endl;
	      cout << bounding_boxes[k] << endl;
	    }
	    
	  }
	}
      }
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
  }
}

template<typename T>
void print_histogram(vector<T>& items) {
  stable_sort(items.begin(), items.end());
  cout << "Total elements: " << items.size() << endl;
  vector<vector<T>> buckets;
  split_by(items, buckets, [](const T& x, const T& y) { return x == y; });
  for (auto b : buckets) {
    cout << b.front() << "\t" << b.size() << endl;
  }
}

void collect_feeds_and_speeds(vector<double>& h,
			      vector<double>& sp,
			      const string& dir_name) {
  if (ends_with(dir_name, ".NCF")) {
    cout << dir_name << endl;
    std::ifstream t(dir_name);
    std::string str((std::istreambuf_iterator<char>(t)),
		    std::istreambuf_iterator<char>());
    vector<block> p = lex_gprog(str);
    cout << "NUM BLOCKS: " << p.size() << endl;
    vector<machine_state> states = all_program_states(p);
    cout << "STATES: " << states.size() << endl;
    for (auto s : states) {
      if (s.feedrate->is_lit() &&
	  s.spindle_speed->is_lit() &&
	  is_cut(s)) {
	h.push_back(static_cast<lit*>(s.feedrate)->v);
	sp.push_back(static_cast<lit*>(s.spindle_speed)->v);
      }
    }
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
  vector<double> feedrates;
  vector<double> spindle_speeds;
  read_dir(dir_name, [&feedrates, &spindle_speeds](const string& file_name)
	   { collect_feeds_and_speeds(feedrates, spindle_speeds, file_name); });
  cout << "FEED RATE INFO" << endl;
  print_histogram(feedrates);
  cout << "SPINDLE SPEED INFO" << endl;
  print_histogram(spindle_speeds);
  time(&end);
  double seconds = difftime(end, start);
  cout << "Total time to process all .NCF files: " << seconds << endl;
}

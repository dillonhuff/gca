#include <cassert>
#include <ctime>
#include <streambuf>

#include "analysis/gcode_to_cuts.h"
#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/profiler.h"
#include "analysis/unfold.h"
#include "analysis/utils.h"
#include "core/lexer.h"
#include "geometry/box.h"
#include "synthesis/cut.h"
#include "synthesis/cut_to_gcode.h"
#include "system/algorithm.h"
#include "system/arena_allocator.h"
#include "system/file.h"

using namespace gca;
using namespace std;

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
      f(p);
    }
  };
  read_dir(dn, func);
}

void split_and_print(const vector<machine_state>& toolpath) {
  auto ptbl = select_column(G54_COORD_SYSTEM, program_position_table(toolpath));
  vector<pair<machine_state, position>> tstates(ptbl.size());
  zip(toolpath.begin(), toolpath.end(), ptbl.begin(), tstates.begin());
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
    cout << "# of Positions in toolpath: " << path.size() << endl;
  }
}

void print_toolpaths(const vector<machine_state>& states) {
  vector<vector<machine_state>> toolpaths;
  split_by(states, toolpaths, [](const machine_state& c, const machine_state& p)
	   { return c.active_tool == p.active_tool; });
  delete_if(toolpaths, [](const vector<machine_state>& c)
	    { return c.back().active_tool->is_omitted(); });
  // On HAAS it appears that the last toolpath is always a change back
  // to the first tool that was active, this toolpath never contains
  // any moves
  if (toolpaths.size() > 0) { toolpaths.pop_back(); };
  cout << "Number of toolpaths with known tool: " << toolpaths.size() << endl;
  for (auto toolpath : toolpaths) {
    if (is_analyzable(toolpath)) {
      split_and_print(toolpath);
    }
  }
}

void print_paths_gcode(vector<vector<cut*>> paths) {
  cut_params params;
  params.target_machine = PROBOTIX_V90_MK2_VFD;
  params.safe_height = 2.0;
  vector<box> path_boxes;
  for (auto path : paths) {
    print_profile_info(path);
    path_boxes.push_back(path_bounds(path));
  }
  cout << "---------------------------------------------------------" << endl;
  cout << "PATH GEOMETRY" << endl;
  cout << "---------------------------------------------------------" << endl;
  if (paths.size() > 0) {
    box bound = bound_boxes(path_boxes);
    cout << "All path bounding box: " << endl << bound << endl;
    box new_machine_bounds(0.5, 17,
			   1.2, 13.2,
			   -4.1, -0.05);
    if (fits_inside(bound, new_machine_bounds)) {
      cout << "FITS NEW MACHINE BOUNDS" << endl;
    } else {
      cout << "DOES NOT FIT NEW MACHINE BOUNDS" << endl;
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
  int num_paths;
  apply_to_gprograms(dir_name, [&num_paths](const vector<block>& p) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {
	cout << "Paths: " << paths.size() << endl;
	num_paths += paths.size();
	print_paths_gcode(paths);
      } else {
	cout << "Could not process all paths: " << r << endl;
      }
    });
  cout << "# paths: " << num_paths << endl;
  time(&end);
  double seconds = difftime(end, start);
  cout << "Total time to process all .NCF files: " << seconds << " seconds" << endl;
}

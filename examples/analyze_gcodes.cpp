#include <cassert>
#include <ctime>
#include <map>
#include <numeric>
#include <streambuf>

#include "analysis/gcode_to_cuts.h"
#include "analysis/machine_state.h"
#include "analysis/position_table.h"
#include "analysis/profiler.h"
#include "analysis/unfold.h"
#include "analysis/utils.h"
#include "backend/cut_to_gcode.h"
#include "backend/output.h"
#include "checkers/block_rate_checker.h"
#include "gcode/lexer.h"
#include "geometry/box.h"
#include "geometry/vtk_debug.h"
#include "simulators/region.h"
#include "simulators/sim_mill.h"
#include "gcode/cut.h"
#include "utils/algorithm.h"
#include "utils/arena_allocator.h"
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

vector<cut*> clip_transition_heights(const vector<cut*>& path,
				     double new_safe_height) {
  vector<vector<cut*>> move_sequences =
    group_unary(path, [](const cut* c) { return c->is_safe_move(); });
  delete_if(move_sequences, [](const vector<cut*>& cs)
	    { return cs.front()->is_safe_move(); });
  bool all_normal_paths = true;
  for (auto s : move_sequences) {
    if (!is_vertical(s.front()) || s.size() < 2) {
      all_normal_paths = false;
      break;
    }
  }
  vector<cut*> clipped_cuts;
  if (all_normal_paths) {
    cout << "All normal paths" << endl;
    vector<vector<cut*>> transitions(move_sequences.size() - 1);
    // TODO: Add push feedrate inference
    auto mk_transition = [new_safe_height](const vector<cut*> l,
					   const vector<cut*> r) {
      return from_to_with_G0_height(l.back()->get_end(),
				    r.front()->get_start(),
				    new_safe_height, lit::make(10.0));
    };
    apply_between(move_sequences.begin() + 1, move_sequences.end(),
		  transitions.begin(),
		  mk_transition);
    for (unsigned i = 0; i < move_sequences.size(); i++) {
      auto m = move_sequences[i];
      clipped_cuts.insert(end(clipped_cuts), m.begin() + 1, m.end());
      if (i < move_sequences.size() - 1) {
	auto t = transitions[i];
	clipped_cuts.insert(end(clipped_cuts), t.begin(), t.end());
      }
    }
  } else {
    clipped_cuts = path;
  }
  return clipped_cuts;
}

vector<vector<cut*>> clip_transition_heights(vector<vector<cut*>>& paths,
					     double new_safe_height) {
  vector<vector<cut*>> clipped_paths;
  for (auto path : paths) {
    clipped_paths.push_back(clip_transition_heights(path, new_safe_height));
  }
  return clipped_paths;
}

void print_geometry_info(vector<vector<cut*>>& paths) {
  vector<box> path_boxes;
  for (auto path : paths) {
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
  if (!all_cuts_within_block_rate(paths, 4000)) {
    cout << "NOT ALL CUTS FIT IN THE BLOCK RATE" << endl;
  }
}

void print_profile_info(vector<vector<cut*>>& paths) {
  for (auto path : paths) {
    print_profile_info(path);
  }
}

void print_performance_diff(const program_profile_info& before,
			    const program_profile_info& after) {
  assert(before.size() == after.size());
  double time_before = execution_time(before);
  double time_after = execution_time(after);
  assert(!within_eps(time_before, 0));
  double pct_change = ((time_before - time_after) / time_before) * 100;
  cout << "Time before  = " << time_before << endl;
  cout << "Time after   = " << time_after << endl;
  cout << "% change     = " << pct_change << endl;;
}

void print_paths_gcode(vector<vector<cut*>>& paths) {
  program_profile_info before = profile_toolpaths(paths);
  vector<vector<cut*>> clipped_height_paths = clip_transition_heights(paths, 0.01);
  program_profile_info after = profile_toolpaths(clipped_height_paths);
  print_performance_diff(before, after);
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

double estimate_feedrate_median(const std::vector<cut*>& path) {
  vector<cut*> actual_cuts =
    select(path, [](const cut* c) { return !c->is_safe_move(); });

  vector<double> feeds;
  for (auto& c : path) {
    feeds.push_back(static_cast<lit*>(c->get_feedrate())->v);
  }

  sort(begin(feeds), end(feeds));

  int ind = feeds.size() / 2;

  return feeds[ind];
}

double chip_load(const double feedrate_ipm,
		 const double spindle_speed,
		 const double num_flutes) {
  return feedrate_ipm / (spindle_speed * num_flutes);
}

double surface_feet_per_minute(const double spindle_speed,
			       const double tool_diameter_inches) {
  double tool_diameter_feet = tool_diameter_inches / 12.0;
  double tool_circumference_feet = M_PI*tool_diameter_feet;

  return tool_circumference_feet * spindle_speed;
}

double estimate_spindle_speed_median(const std::vector<cut*>& path) {
  vector<cut*> actual_cuts =
    select(path, [](const cut* c) { return !c->is_safe_move(); });

  vector<double> speeds;
  for (auto& c : path) {
    speeds.push_back(static_cast<lit*>(c->get_spindle_speed())->v);
  }

  sort(begin(speeds), end(speeds));

  int ind = speeds.size() / 2;

  return speeds[ind];
}

double estimate_cut_depth_median(const std::vector<cut*>& path) {
  vector<cut*> actual_cuts =
    select(path, [](const cut* c) { return !c->is_safe_move(); });

  if (path.size() < 2) { return -1.0; }

  vector<double> diffs(actual_cuts.size() - 1);
  apply_between(begin(actual_cuts), end(actual_cuts), begin(diffs),
		[](const cut* l, const cut* r) {
		  return fabs(l->get_end().z - r->get_end().z);
		});

  delete_if(diffs, [](const double d) { return within_eps(d, 0.0, 1e-3); });

  if (diffs.size() < 2) { return -1.0; }

  // for (auto& d : diffs) {
  //   cout << d << endl;
  // }

  sort(begin(diffs), end(diffs));

  int ind = diffs.size() / 2;
  return diffs[ind];
}

double estimate_cut_depth_mean(const std::vector<cut*>& path) {
  vector<cut*> actual_cuts =
    select(path, [](const cut* c) { return !c->is_safe_move(); });

  if (path.size() < 2) { return -1.0; }

  vector<double> diffs(actual_cuts.size() - 1);
  apply_between(begin(actual_cuts), end(actual_cuts), begin(diffs),
		[](const cut* l, const cut* r) {
		  return fabs(l->get_end().z - r->get_end().z);
		});

  delete_if(diffs, [](const double d) { return within_eps(d, 0.0, 1e-3); });

  // for (auto& d : diffs) {
  //   cout << d << endl;
  // }

  double depth = accumulate(begin(diffs), end(diffs), 0.0);
  depth = depth / diffs.size();

  return depth;
}

void simulate_paths(vector<vector<cut*>>& paths,
		    map<int, double>& tool_table,
		    vector<double>& mrrs) {
  if (paths.size() == 0) { return; }
  // TODO: Add proper tool diameter max checking

  double max_tool_diameter = 1.5;
  // auto r = set_up_region_conservative(paths, max_tool_diameter);

  // vtk_debug_depth_field(r.r);

  for (auto path : paths) {
    cout << "Looking up tool diameter" << endl;
    auto c = *find_if(path.begin(), path.end(),
		       [](const cut* c) { return !c->is_safe_move(); });
    auto tn = c->settings.active_tool; //path.front()->settings.active_tool;
    if (!(tn->is_ilit())) {
      cout << "ERROR" << endl;
      cout << *c << endl;
      cout << "Active tool = " << *(c->settings.active_tool) << endl;
      assert(false);
    }
    auto tl = static_cast<ilit*>(tn);
    int current_tool_no = tl->v;
    double tool_diameter = tool_table[current_tool_no]; //0.125;
    cylindrical_bit t = (tool_diameter);

    double cut_depth = estimate_cut_depth_median(path);
    double feedrate = estimate_feedrate_median(path);
    double spindle_speed = estimate_spindle_speed_median(path);
    double sfm = surface_feet_per_minute(spindle_speed, tool_diameter);

    double cl_2_flute = chip_load(spindle_speed, feedrate, 2);
    double cl_4_flute = chip_load(spindle_speed, feedrate, 4);
    double cl_6_flute = chip_load(spindle_speed, feedrate, 6);

    cout << "--------------------------------------------------------" << endl;

    cout << "current_tool_no = " << current_tool_no << endl;
    cout << "Tool diameter = " << tool_diameter << endl << endl;

    cout << "cut depth estimate = " << cut_depth << endl;
    cout << "feedrate estimate = " << feedrate << endl;
    cout << "spindle speed estimate = " << spindle_speed << endl << endl;
    
    cout << "implied sfm = " << sfm << endl;

    cout << "implied CL for 2 flutes = " << cl_2_flute << endl;
    cout << "implied CL for 4 flutes = " << cl_4_flute << endl;
    cout << "implied CL for 6 flutes = " << cl_6_flute << endl;
    
    cout << "--------------------------------------------------------" << endl;

    

    // for (auto c : path) {
    //   double volume_removed = update_cut(*c, r, t);
    //   double execution_time = cut_execution_time_minutes(c);

    //   if (!within_eps(execution_time, 0.0)) {

    // 	if (!c->is_safe_move()) {
    // 	  auto f = c->get_feedrate();
    // 	  auto sp = c->get_spindle_speed();

    // 	  double cut_length = (c->get_end() - c->get_start()).len();
    // 	  double mrr = volume_removed / execution_time;
	  
    // 	  // cout << "Feedrate       = " << static_cast<lit*>(f)->v << endl;
    // 	  // cout << "Spindle speed  = " << static_cast<lit*>(sp)->v << endl;
    // 	  // cout << "Z start        = " << c->get_start().z << endl;
    // 	  // cout << "Z end        = " << c->get_end().z << endl;
	  
    // 	  // cout << "Volume removed = " << volume_removed << endl;
    // 	  // cout << "Cut length     = " << cut_length << endl;
    // 	  // cout << "MRR            = " << mrr << endl;

    // 	  mrrs.push_back(mrr);
    // 	}

    // 	// if (c->is_safe_move()) {
    // 	//   cout << "SAFE MOVE WITH MRR = " << mrr << endl;
	  
    // 	//   DBG_ASSERT(false);
    // 	// }


    //   }

    // }

    // vtk_debug_depth_field(r.r);


  }

  // auto mm = minmax_element(mrrs.begin(), mrrs.end());
  // auto total_removed = accumulate(mrrs.begin(), mrrs.end(), 0.0);
  // auto cut_average_mrr = total_removed / static_cast<double>(mrrs.size());

  // cout << "MRR STATS" << endl;
  // cout << "-----------------------------------------------------" << endl;
  // cout << "Average MRR so far  = "<< cut_average_mrr << endl;
  // cout << "Smallest MRR so far = " << *mm.first << endl;
  // cout << "Largest MRR so far  = " << *mm.second << endl;
  // cout << "-----------------------------------------------------" << endl;
}

bool starts_with(string& value, string& prefix) {
  if (prefix.size() > value.size()) return false;
  auto res = std::mismatch(prefix.begin(), prefix.end(), value.begin());
  return res.first == prefix.end();
}

void add_tool(map<int, double>& tt, string& comment) {
  string tool_comment_start = "( TOOL ";
  if (starts_with(comment, tool_comment_start)) {
    cout << "Tool comment is " << comment << endl;
    size_t i = -1;
    int tool_no = stoi(comment.substr(tool_comment_start.size()), &i);
    cout << "tool_no = " << tool_no << endl;
    assert(i != -1);
    string rest = comment.substr(tool_comment_start.size() + i);
    cout << "Rest of comment = " << rest << endl;
    double tool_diameter = stod(rest);
    cout << "tool diameter = " << tool_diameter << endl;
    tt[tool_no] = tool_diameter;
  }
}

map<int, double> infer_tool_table(const vector<block>& p) {
  vector<token> comments;
  for (auto b : p) {
    for (auto t : b) {
      if ((t.ttp == PAREN_COMMENT) ||
	  (t.ttp == BRACKET_COMMENT)) {
	comments.push_back(t);
      }
    }
  }
  map<int, double> tt;
  for (auto c : comments) {
    add_tool(tt, c.text);
  }
  return tt;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: analyze-gcodes <directory path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];
  time_t start;
  time_t end;
  time(&start);
  int num_paths;
  vector<double> mrrs;
  apply_to_gprograms(dir_name, [&num_paths, &mrrs](const vector<block>& p) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {
	map<int, double> tt = infer_tool_table(p);
	simulate_paths(paths, tt, mrrs);
      } else {
	cout << "Could not process all paths: " << r << endl;
      }
    });
  double num_large_mrrs = count_if(mrrs.begin(), mrrs.end(),
				   [](double mrr) { return mrr > 5.0; });;
  cout << "# paths: " << num_paths << endl;
  cout << "# files w/ MRR > 10 in^3/min: " << num_large_mrrs << endl;
  time(&end);
  double seconds = difftime(end, start);
  cout << "Total time to process all .NCF files: " << seconds << " seconds" << endl;

  cout << "mrrs.size() = " << mrrs.size() << endl;
  //print_histogram(mrrs);
}

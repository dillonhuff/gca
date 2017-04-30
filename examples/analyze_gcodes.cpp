#include <cassert>
#include <ctime>
#include <future>
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
#include "simulators/simulate_operations.h"
#include "synthesis/visual_debug.h"
#include "gcode/cut.h"
#include "utils/algorithm.h"
#include "utils/grouping.h"
#include "utils/arena_allocator.h"
#include "system/file.h"
#include "system/json.h"

using namespace gca;
using namespace std;

int num_unsafe_moves(const simulation_log& l);

using int_futures = std::vector<std::future<int>>;

int accumulate_block_worker_ret(string* data, size_t count) {
  return 0; //std::accumulate(data, data + count, 0);
}

int apply_to_each(std::vector<string> const * const v,
		  const unsigned start,
		  const unsigned end) {

  //  for (auto& dir_name : *v) {
  for (unsigned i = start; i < end; i++) {
    string dir_name = (*v)[i];

    std::ifstream t(dir_name);
    std::string str((std::istreambuf_iterator<char>(t)),
		    std::istreambuf_iterator<char>());
    vector<block> p = lex_gprog(str);
    cout << "NUM BLOCKS: " << p.size() << endl;

    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(p, paths);
    if (r == GCODE_TO_CUTS_SUCCESS) {

      // map<int, tool_info> tt = infer_tool_table_GCA(p);

      // std::vector<operation_range> op_ranges =
      // 	infer_operation_ranges_GCA(p);

      // simulation_log l = simulation_log_GCA(paths, tt, op_ranges);

      // int num_unsafe_G0s = num_unsafe_moves(l);

      // cout << "# of unsafe moves = " << num_unsafe_G0s << endl;

    }
  
  }
  return 0;
}

int_futures launch_split_workers_with_std_async(std::vector<string>& v) {
  unsigned split = v.size() / 2;

  int_futures futures;
  futures.push_back(std::async(std::launch::async, apply_to_each,
			       &v, 0, split));

  futures.push_back(std::async(std::launch::async, apply_to_each,
			       &v, split, v.size()));
  
  // int_futures futures;
  // futures.push_back(std::async(std::launch::async, accumulate_block_worker_ret,
  //                              v.data(), v.size() / 2));
  // futures.push_back(std::async(std::launch::async, accumulate_block_worker_ret,
  //                              v.data() + v.size() / 2, v.size() / 2));
  return futures;
}

template<typename F>
void apply_to_gprograms_parallel(const string& dn, F f) {

  std::vector<std::string> names;

  auto func = [&names](const string& dir_name) {
    if (ends_with(dir_name, ".NCF")) {
      cout << dir_name << endl;
      names.push_back(dir_name);
    }
  };
  read_dir(dn, func);

  // cout << "# of names = " << names.size() << endl;
  // for (auto& n : names) {
  //   cout << n << endl;
  // }

  auto fs = launch_split_workers_with_std_async(names);
  int total = 0;
  for (auto& f : fs) {
    total += f.get();
  }

  cout << "total = " << total << endl;
  
  // auto func = [&f](const string& dir_name) {
  //   if (ends_with(dir_name, ".NCF")) {
  //     cout << dir_name << endl;
  //     std::ifstream t(dir_name);
  //     std::string str((std::istreambuf_iterator<char>(t)),
  // 		      std::istreambuf_iterator<char>());
  //     vector<block> p = lex_gprog(str);
  //     cout << "NUM BLOCKS: " << p.size() << endl;
  //     f(p, dir_name);
  //   }
  // };
  // read_dir(dn, func);
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
      f(p, dir_name);
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

// void simulate_paths(vector<vector<cut*>>& paths,
// 		    map<int, tool_info>& tool_table,
// 		    vector<double>& mrrs) {
//   if (paths.size() == 0) { return; }
//   // TODO: Add proper tool diameter max checking

//   double max_tool_diameter = 1.5;
//   // auto r = set_up_region_conservative(paths, max_tool_diameter);

//   // vtk_debug_depth_field(r.r);

//   for (auto path : paths) {
//     cout << "Looking up tool diameter" << endl;
//     auto c = *find_if(path.begin(), path.end(),
// 		       [](const cut* c) { return !c->is_safe_move(); });
//     auto tn = c->settings.active_tool; //path.front()->settings.active_tool;
//     if (!(tn->is_ilit())) {
//       cout << "ERROR" << endl;
//       cout << *c << endl;
//       cout << "Active tool = " << *(c->settings.active_tool) << endl;
//       assert(false);
//     }
//     auto tl = static_cast<ilit*>(tn);
//     int current_tool_no = tl->v;
//     double tool_diameter = tool_table[current_tool_no].tool_diameter;
//     cylindrical_bit t = (tool_diameter);

//     double cut_depth = estimate_cut_depth_median(path);
//     double feedrate = estimate_feedrate_median(path);
//     double spindle_speed = estimate_spindle_speed_median(path);
//     double sfm = surface_feet_per_minute(spindle_speed, tool_diameter);

//     double cl_2_flute = chip_load(spindle_speed, feedrate, 2);
//     double cl_4_flute = chip_load(spindle_speed, feedrate, 4);
//     double cl_6_flute = chip_load(spindle_speed, feedrate, 6);

//     cout << "--------------------------------------------------------" << endl;

//     cout << "current_tool_no = " << current_tool_no << endl;
//     cout << "Tool diameter = " << tool_diameter << endl << endl;

//     cout << "cut depth estimate = " << cut_depth << endl;
//     cout << "feedrate estimate = " << feedrate << endl;
//     cout << "spindle speed estimate = " << spindle_speed << endl << endl;
    
//     cout << "implied sfm = " << sfm << endl;

//     cout << "implied CL for 2 flutes = " << cl_2_flute << endl;
//     cout << "implied CL for 4 flutes = " << cl_4_flute << endl;
//     cout << "implied CL for 6 flutes = " << cl_6_flute << endl;
    
//     cout << "--------------------------------------------------------" << endl;

    

//     // for (auto c : path) {
//     //   double volume_removed = update_cut(*c, r, t);
//     //   double execution_time = cut_execution_time_minutes(c);

//     //   if (!within_eps(execution_time, 0.0)) {

//     // 	if (!c->is_safe_move()) {
//     // 	  auto f = c->get_feedrate();
//     // 	  auto sp = c->get_spindle_speed();

//     // 	  double cut_length = (c->get_end() - c->get_start()).len();
//     // 	  double mrr = volume_removed / execution_time;
	  
//     // 	  // cout << "Feedrate       = " << static_cast<lit*>(f)->v << endl;
//     // 	  // cout << "Spindle speed  = " << static_cast<lit*>(sp)->v << endl;
//     // 	  // cout << "Z start        = " << c->get_start().z << endl;
//     // 	  // cout << "Z end        = " << c->get_end().z << endl;
	  
//     // 	  // cout << "Volume removed = " << volume_removed << endl;
//     // 	  // cout << "Cut length     = " << cut_length << endl;
//     // 	  // cout << "MRR            = " << mrr << endl;

//     // 	  mrrs.push_back(mrr);
//     // 	}

//     // 	// if (c->is_safe_move()) {
//     // 	//   cout << "SAFE MOVE WITH MRR = " << mrr << endl;
	  
//     // 	//   DBG_ASSERT(false);
//     // 	// }


//     //   }

//     // }

//     // vtk_debug_depth_field(r.r);


//   }

//   // auto mm = minmax_element(mrrs.begin(), mrrs.end());
//   // auto total_removed = accumulate(mrrs.begin(), mrrs.end(), 0.0);
//   // auto cut_average_mrr = total_removed / static_cast<double>(mrrs.size());

//   // cout << "MRR STATS" << endl;
//   // cout << "-----------------------------------------------------" << endl;
//   // cout << "Average MRR so far  = "<< cut_average_mrr << endl;
//   // cout << "Smallest MRR so far = " << *mm.first << endl;
//   // cout << "Largest MRR so far  = " << *mm.second << endl;
//   // cout << "-----------------------------------------------------" << endl;
// }

std::string to_string(const operation_type op_type) {
  switch (op_type) {

  case ROUGH_OPERATION:
    return "ROUGH_OPERATION";

  case FINISH_OPERATION:
    return "FINISH_OPERATION";

  case OTHER_OPERATION:
    return "OTHER_OPERATION";
  }

  DBG_ASSERT(false);
}

struct labeled_operation_params {
  operation_type op_type;
  operation_params params;
};

ptree encode_json(const operation_range& op_range) {
  ptree p;
  p.put("name", op_range.name);
  p.put("start_line", op_range.start_line);
  p.put("end_line", op_range.end_line);

  return p;
}

ptree encode_json(const operation_params& op) {
  ptree p;
  p.put("current_tool_no", op.current_tool_no);
  p.put("tool_end_type", to_string(op.tool_end_type));
  p.put("tool_diameter", op.tool_diameter);
  p.put("cut_depth", op.cut_depth);

  p.put("feedrate", op.feedrate);
  p.put("spindle_speed", op.spindle_speed);
  // IS SFM parameter even needed?
  p.put("sfm", op.sfm);

  p.put("total_distance", op.total_distance);
  p.put("cut_distance", op.cut_distance);

  p.put("total_time", op.total_time);
  p.put("cut_time", op.cut_time);

  p.put("material_removed", op.material_removed);

  p.put("file_name", op.file_name);

  p.push_back( make_pair("range", encode_json(op.range)) );
  
  // ptree tn;
  // tn.put("", op.current_tool_no);
  // p.add_child("current_tool_no", tn);
  
  return p;
}

ptree encode_json(const labeled_operation_params& op) {
  
  ptree params = encode_json(op.params);

  ptree enc;
  enc.put("op_type", to_string(op.op_type));
  enc.add_child("params", params);

  return enc;
}

struct op_replacement {
  operation_params worse;
  operation_params better;
};

std::vector<polyline> cuts_to_polylines(const std::vector<cut*>& cuts) {
  vector<point> points;
  for (auto& c : cuts) {
    if (c->is_safe_move() || c->is_linear_cut()) {
      points.push_back(c->get_start());
      points.push_back(c->get_end());
    } else {
      double ind = 0;
      while (ind < 1) {
	points.push_back(c->value_at(ind));
	ind += 0.1;
      }
      points.push_back(c->get_end());
    }
  }
  return {{points}};
}

void vtk_debug_cuts(const std::vector<cut*>& cuts) {
  vector<polyline> lines = cuts_to_polylines(cuts);
  auto pd = polydata_for_polylines(lines);
  visualize_polydatas({pd});
}

tool_end decode_tool_end_json(const ptree& p) {
  string tool_name = p.get<std::string>("");

  if (tool_name == "ROUGH_ENDMILL") {
    return ROUGH_ENDMILL;
  }

  if (tool_name == "BALL_ENDMILL") {
    return BALL_ENDMILL;
  }

  if (tool_name == "FINISH_ENDMILL") {
    return FINISH_ENDMILL;
  }

  if (tool_name == "SPOT_DRILL") {
    return SPOT_DRILL_ENDMILL;
  }

  if (tool_name == "COUNTERSINK") {
    return COUNTERSINK_ENDMILL;
  }

  if (tool_name == "DRILL") {
    return DRILL_ENDMILL;
  }

  if (tool_name == "FACE") {
    return FACE_ENDMILL;
  }

  if (tool_name == "KEY_CUTTER") {
    return KEY_CUTTER_ENDMILL;
  }

  if (tool_name == "FLY_CUTTER") {
    return FLY_CUTTER_ENDMILL;
  }
  
  
  DBG_ASSERT(false);
}

operation_params decode_json_params(const ptree& p) {
  int ctn = decode_json<int>(p.get_child("current_tool_no"));

  // p.put("tool_end_type", to_string(op.tool_end_type));
  tool_end tet = decode_tool_end_json(p.get_child("tool_end_type"));

  // p.put("tool_diameter", op.tool_diameter);
  double tool_diam = decode_json<double>(p.get_child("tool_diameter"));

  // p.put("cut_depth", op.cut_depth);
  double cut_depth = decode_json<double>(p.get_child("cut_depth"));

  // p.put("spindle_speed", op.spindle_speed);
  double feedrate = decode_json<double>(p.get_child("feedrate"));
  
  // p.put("spindle_speed", op.spindle_speed);
  double spindle_speed = decode_json<double>(p.get_child("spindle_speed"));

  // // IS SFM parameter even needed?
  // p.put("sfm", op.sfm);
  double sfm = decode_json<double>(p.get_child("sfm"));

  // p.put("total_distance", op.total_distance);
  double total_distance = decode_json<double>(p.get_child("total_distance"));

  // p.put("cut_distance", op.cut_distance);
  double cut_distance = decode_json<double>(p.get_child("cut_distance"));

  // p.put("total_time", op.total_time);
  double total_time = decode_json<double>(p.get_child("total_time"));

  // p.put("cut_time", op.cut_time);
  double cut_time = decode_json<double>(p.get_child("cut_time"));

  // p.put("material_removed", op.material_removed);
  double material_removed = decode_json<double>(p.get_child("material_removed"));

  // p.put("file_name", op.file_name);
  string file_name = p.get<std::string>("file_name");

  ptree range_pt = p.get_child("range");
  operation_range range{range_pt.get<std::string>("name"),
      range_pt.get<int>("start_line"),
      range_pt.get<int>("end_line")};

  operation_params op{ctn,
      tet,
      tool_diam,
      cut_depth,
      feedrate,
      spindle_speed,
      sfm,
      total_distance,
      cut_distance,
      total_time,
      cut_time,
      material_removed,
      file_name,
      range};

    return op;
}

std::vector<operation_params> decode_params(const ptree& p) {
  std::vector<operation_params> elems;
  BOOST_FOREACH(const ptree::value_type& v, p.get_child("")) {
    elems.push_back(decode_json_params(v.second));
  }
  return elems;
}

void print_sfm_mrr_csv(const std::vector<operation_params>& likely_rough_ops) {
  for (auto& op : likely_rough_ops) {
    cout << op.SFM() << "," << op.average_MRR() << endl;
  }
}

std::vector<operation_params>
simulate_program_GCA(const vector<block>& p, const string& file_name) {
  vector<vector<cut*>> paths;
  auto r = gcode_to_cuts(p, paths);
  if (r == GCODE_TO_CUTS_SUCCESS) {
    map<int, tool_info> tt = infer_tool_table_GCA(p);

    std::vector<operation_range> op_ranges =
      infer_operation_ranges_GCA(p);

    vector<operation_params> prog_ops =
      program_operations_GCA(paths, tt, op_ranges);

    return prog_ops;
  } else {
    cout << "Could not parse operation " << r << endl;
    return {};
  }

}

void simulate_all_programs(const std::string& dir_name) {
  int num_paths;

  vector<double> mrrs;

  vector<operation_params> all_params;
  int num_processed_blocks = 0;
  int num_failed_blocks = 0;

  apply_to_gprograms(dir_name, [&all_params, &num_processed_blocks, &num_failed_blocks](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {
  	num_processed_blocks += p.size();

  	map<int, tool_info> tt = infer_tool_table_HAAS(p);

  	std::vector<operation_range> op_ranges =
  	  infer_operation_ranges_HAAS(p);

  	vector<operation_params> prog_ops =
  	  program_operations_HAAS(paths, tt, op_ranges);

  	double program_length = 0.0;
  	double program_cut_length = 0.0;
  	double program_cut_time = 0.0;
  	for (auto& op : prog_ops) {
  	  program_length += op.total_distance;
  	  program_cut_length += op.cut_distance;
  	  program_cut_time += op.cut_time;
  	}

  	cout << "Program length in feet = " << program_length / 12.0 << endl;
  	boost::optional<double> stated_len =
  	  infer_program_length_feet(p);

  	if (stated_len) {
  	  cout << "STATED program length in feet = " << *stated_len << endl;
  	}

  	for (auto& op : prog_ops) {
  	  op.file_name = file_name;
  	}

  	concat(all_params, prog_ops);

  	//simulate_paths(paths, tt, mrrs);
      } else {
  	cout << "Could not process all paths: " << r << endl;
  	num_failed_blocks += p.size();
      }
    });

  cout << "# processed files = " << num_processed_blocks << endl;
  cout << "# of failed files = " << num_failed_blocks << endl;
  cout << "fraction processed = " << static_cast<double>(num_processed_blocks) / static_cast<double>(num_processed_blocks + num_failed_blocks) << endl;
  cout << "# of operations = " << all_params.size() << endl;

  // ptree all_params_json_arr = encode_json(all_params);

  // ptree all_params_json;
  // all_params_json.add_child("All params", all_params_json_arr);

  // cout << "ALL PARAMS AS JSON" << endl;
  // write_json(cout, all_params_json);

}

int num_unsafe_moves(const simulation_log& l) {
  int num_unsafe_G0s = 0;

  for (auto& op : l.operation_logs) {
    for (auto& cut_log : op.cuts) {
      cut* c = cut_log.c;
      const vector<point_update>& updates = cut_log.updates;
      vector<grid_update> total_updates =
	sum_updates(updates);

      if (c->is_safe_move() &&
	  !(is_vertical(c) && ( c->get_start().z < c->get_end().z ))) {

	double vol_removed =
	  volume_removed_in_updates(l.resolution, total_updates);

	if (vol_removed > 0.0001) {
	  cout << "Volume removed in safe move!" << endl;
	  cout << *c << endl;
	  cout << "Volume removed = " << vol_removed << endl;
	  cout << "Line number = " << c->get_line_number() << endl;
	  num_unsafe_G0s++;
	}
      }
    }
  }

  return num_unsafe_G0s;
}

ptree
encode_json(const tool_info& op_info) {
  ptree p;
  p.put("tool_end_type", to_string(op_info.tool_end_type));
  p.put("tool_diameter", op_info.tool_diameter);
  return p;
}

ptree
encode_json(const operation_info& op_info) {
  ptree p;
  p.add_child("range", encode_json(op_info.range));
  p.add_child("tool_inf", encode_json(op_info.tool_inf));
  return p;
}

ptree
encode_json(const grid_cell& cell) {
  ptree p;
  p.put("x_ind", cell.x_ind);
  p.put("y_ind", cell.y_ind);

  return p;
}

ptree
encode_json(const grid_update& cell) {
  ptree p;

  p.add_child("cell", encode_json(cell.cell));
  p.put("height_diff", cell.height_diff);

  return p;
}

ptree
encode_json(const point_update& cut_log) {
  ptree p;
  p.add_child("cutter_location", encode_json(cut_log.cutter_location));

  ptree grid_updates;
  for (auto& g : cut_log.grid_updates) {
    grid_updates.push_back( make_pair("", encode_json(g)) );
  }

  p.add_child("grid_updates", grid_updates);

  return p;
}

ptree
encode_json(const cut_simulation_log& cut_log) {
  ptree p;

  ptree updates;

  if (cut_log.updates.size() > 0) {
    vector<grid_update> sum = sum_updates(cut_log.updates);
    // Dummy cutter location
    point start = cut_log.updates.front().cutter_location;
    point_update all{start, sum};

    updates.push_back( make_pair("", encode_json(all)) );
    
    // for (auto& pu : cut_log.updates) {
    //   updates.push_back( make_pair("", encode_json(pu)) );
    // }
  }

  p.add_child("updates", updates);
  return p;
}

ptree
encode_json(const operation_log& op_log) {
  ptree p;
  p.add_child("info", encode_json(op_log.info));

  ptree cuts;
  for (auto& cut_log : op_log.cuts) {
    cuts.push_back( make_pair("", encode_json(cut_log)) );
  }

  p.add_child("cuts", cuts);
  
  return p;
}

ptree
encode_json(const simulation_log& sim_log) {
  ptree p;
  p.put("resolution", sim_log.resolution);

  ptree children;
  for (auto& op : sim_log.operation_logs) {
    children.push_back( make_pair("", encode_json(op)) );
  }

  p.add_child("operations", children);
  return p;
}

ptree encode_params(const std::vector<operation_params>& elems) {
  ptree children;
  for (auto e : elems) {
    children.push_back(std::make_pair("", encode_json(e)));
  }
  return children;
}

void
write_logs_to_json(const std::vector<pair<string, simulation_log> >& file_log_pairs) {
  ptree p;

  ptree children;
  for (auto& file_log_pair : file_log_pairs) {
    ptree c;
    c.put("name", file_log_pair.first);
    c.add_child("log", encode_json(file_log_pair.second));
    children.push_back( make_pair("", c) );
  }

  p.add_child("All-ops", children);

  write_json(cout, p);
}

bool program_in_HAAS_travel(const std::vector<std::vector<cut*> >& paths) {
  double HAAS_x_travel = 20;
  double HAAS_y_travel = 16;
  double HAAS_z_travel = 20;

  box bound = bound_paths(paths);

  if ((bound.x_len() > HAAS_x_travel) ||
      (bound.y_len() > HAAS_y_travel) ||
      (bound.z_len() > HAAS_z_travel)) {
    cout << "Box" << endl;
    cout << bound << endl;
    cout << "goes beyond HAAS bounds" << endl;

    return true;
  }

  return false;
}

std::vector<operation_params>
read_operation_params_json(const std::string& dir_name) {
  ptree json_ops;
  read_json(dir_name, json_ops);

  vector<operation_params> read_params =
    decode_params(json_ops.get_child("All params"));

  return read_params;
}

void write_as_json(const std::vector<operation_params>& all_params) {

  ptree all_params_json;
  ptree all_params_json_arr = encode_params(all_params);
  all_params_json.add_child("All params", all_params_json_arr);

  cout << "ALL PARAMS AS JSON" << endl;
  write_json(cout, all_params_json);

}

struct mutated_test_case {
  std::vector<vector<cut*> > paths;
  bool introduced_error;
  bool found_error;
};

bool randomly_mutate(std::vector<std::vector<cut*> >& paths) {
  if (paths.size() == 0) { return false; }

  double r = ((double) rand() / (RAND_MAX));
  if (r < 0.5) {
    cut* last = paths.back().back();
    linear_cut* error_cut =
      new (allocate<linear_cut>()) linear_cut(last->get_end(),
					      point(1000, 1000, 1000));
    paths.back().push_back(error_cut);

    return true;
  }
  return false;
}

void test_mutated_cases(const std::string& dir_name) {
  std::vector<mutated_test_case> cases;

  apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

	bool introduced_error =
	  randomly_mutate(paths);

	bool any_travel_errors = program_in_HAAS_travel(paths);

	if (!introduced_error) {
	  cases.push_back({paths, false, any_travel_errors});
	} else {
	  cases.push_back({paths, true, any_travel_errors});
	}
      }
    });

  int true_positives = 0;
  int true_negatives = 0;
  int false_positives = 0;
  int false_negatives = 0;

  for (auto& c : cases) {
    if (c.introduced_error && c.found_error) {
      true_positives++;
    }

    if (c.introduced_error && !c.found_error) {
      false_negatives++;
    }

    if (!c.introduced_error && c.found_error) {
      false_positives++;
    }
    
    if (!c.introduced_error && !c.found_error) {
      true_negatives++;
    }

  }

  cout << "False positives = " << false_positives << endl;
  cout << "False negatives = " << false_negatives << endl;
  cout << "True positives  = " << true_positives << endl;
  cout << "True negatives  = " << true_negatives << endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: analyze-gcodes <directory path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];
  test_mutated_cases(dir_name);

  return 0;

  vector<operation_params> all_params;

  std::vector<pair<string, simulation_log> > files_to_simulation_logs;
  std::vector<pair<string, int> > files_to_unsafe_moves;

  apply_to_gprograms(dir_name, [&all_params, &files_to_unsafe_moves](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

	bool any_travel_errors = program_in_HAAS_travel(paths);
	
	// map<int, tool_info> tt = infer_tool_table_GCA(p);

  	// std::vector<operation_range> op_ranges =
  	//   infer_operation_ranges_GCA(p);

	// auto ops = program_operations_GCA(paths, tt, op_ranges);
	// concat(all_params, ops);

	//simulation_log l = simulation_log_GCA(paths, tt, op_ranges);

	//files_to_simulation_logs.push_back( make_pair(file_name, prog_ops) );

	//cout << "# of operations = " << l.operation_logs.size() << endl;

	// map<int, tool_info> tt = infer_tool_table_HAAS(p);

  	// std::vector<operation_range> op_ranges =
  	//   infer_operation_ranges_HAAS(p);

	// auto prog_ops = program_operations_HAAS(paths, tt, op_ranges);

	// simulation_log l = simulation_log_HAAS(paths, tt, op_ranges);

	// int num_unsafe_G0s = num_unsafe_moves(l);

	// cout << "# of unsafe moves = " << num_unsafe_G0s << endl;
	
	// files_to_unsafe_moves.push_back( make_pair(file_name, num_unsafe_G0s) );
	
	// for (auto& op : prog_ops) {
	//   cout << "-------------------------------------------------" << endl;
	//   cout << op << endl;
	// }

  	// double program_length = 0.0;
  	// double program_cut_length = 0.0;
  	// double program_cut_time = 0.0;
  	// for (auto& op : prog_ops) {
  	//   program_length += op.total_distance;
  	//   program_cut_length += op.cut_distance;
  	//   program_cut_time += op.cut_time;
  	// }

  	// cout << "Program length in feet = " << program_length / 12.0 << endl;
  	// boost::optional<double> stated_len =
  	//   infer_program_length_feet(p);

  	// if (stated_len) {
  	//   cout << "STATED program length in feet = " << *stated_len << endl;
  	// }

  	// for (auto& op : prog_ops) {
  	//   op.file_name = file_name;
	//   cout << op << endl;
  	// }
	

  	//concat(all_params, prog_ops);

  	//simulate_paths(paths, tt, mrrs);
      } else {
  	cout << "Could not process all paths: " << r << endl;
      }
    });

  //write_as_json(all_params);

  // cout << "Files unsafe moves" << endl;
  // for (auto& file_unsafe_moves_pair : files_to_unsafe_moves) {
  //   cout << file_unsafe_moves_pair.first << " = " << file_unsafe_moves_pair.second << endl;
  // }

  // cout << "All MRRs of programs" << endl;

  // for (auto& op : all_params) {

  //   cout << op.average_MRR() << endl;

  // }

  //  return 0;

  //write_logs_to_json(files_to_simulation_logs);

  //return 0;

  return 0;

  // for (auto& op : p) {
  //   cout << "-------------------------------------------------------------" << endl;
  //   cout << op << endl;
  // }

  // Now start analyzing the trace
  //return 0;

  // time_t start_time;
  // time_t end_time;
  // time(&start_time);

  // return 0;

  // vector<operation_params> likely_rough_ops = read_params;
  // delete_if(likely_rough_ops,
  // 	    [](const operation_params& op) {
  // 	      return //!(op.tool_end_type == ROUGH_ENDMILL) ||
  // 		(op.range.name != "ROUGHING") ||
  // 		within_eps(op.tool_diameter, 0.0, 0.0001) ||
  // 		(op.cut_depth < 0.0); //op.cut_depth < 0.0 || op.material_removed < 0.1;
  // 	    });

  // cout << "# of likely rough operations = " << likely_rough_ops.size() << endl;

  // print_sfm_mrr_csv(likely_rough_ops);

  // return 0;

  // sort_lt(likely_rough_ops, [](const operation_params& l) {
  //     return l.average_MRR();
  //   });

  // for (auto& op : likely_rough_ops) {
  //   cout << endl << "-------------------------------------" << endl;
  //   cout << op << endl;
  // }

  // return 0;

  // vector<vector<operation_params> > grouped =
  //   group_by(likely_rough_ops, [](const operation_params& l,
  // 				  const operation_params& r) {
  // 	       return within_eps(l.tool_diameter, r.tool_diameter, 0.001);
  // 	     });

  // sort(begin(grouped), end(grouped), [](const std::vector<operation_params>& l,
  // 					const std::vector<operation_params>& r) {
  // 	 return l.front().tool_diameter < r.front().tool_diameter;
  //      });

  // cout << "# of tool groups = " << grouped.size() << endl;
  // for (auto& g : grouped) {
  //   cout << "# of operations with diameter " << g.front().tool_diameter;
  //   cout << " inches = " << g.size() << endl;

  //   sort(begin(g), end(g), [](const operation_params& l,
  // 			      const operation_params& r) {
  // 	   return l.average_MRR() < r.average_MRR();
  // 	 });


  //   cout << endl << "&&&&&&&&&& MRRS for Diam = " << g.front().tool_diameter << " &&&&&&&&&&" << endl;
  //   for (auto& op : g) {

  //     cout << "---------------------------------------" << endl;

  //     cout << ", file = " << op.file_name << endl;      
  //     cout << "Type = " << op.tool_end_type << ", Diam = " << op.tool_diameter << endl;
  //     cout << "Speed = " << op.spindle_speed << ", Feed = " << op.feedrate << ", DOC = " << op.cut_depth << ", SFM = " << op.SFM() << endl;
  //     cout << "Material removed = " << op.material_removed << endl;
  //     cout << "average MRR = " << op.average_MRR() << endl;
  //   }
    
    // cout << "SURFACE FEET PER MINUTE" << endl;
    // for (auto op : g) {
    //   cout << "Diam = " << op.tool_diameter << ", Speed = " << op.spindle_speed << ", Feed = " << op.feedrate << ", DOC = " << op.cut_depth << ", SFM = " << op.SFM() << endl;
    //   cout << ", file = " << op.file_name << endl;
    // }

    // auto similar_groups =
    //   group_by(g, [](const operation_params& l,
    // 		     const operation_params& r) {
    // 		 return within_eps(l.SFM(), r.SFM(), 10.0) &&
    // 		 within_eps(l.cut_depth, r.cut_depth, 0.001);
    // 	       });

    // for (auto sim_group : similar_groups) {
    //   if (sim_group.size() > 1) {

    // 	cout << endl << "&&&&&&&&&&&&&&&&&&& CLOSE SFM AND DOC &&&&&&&&&&&&&&&&&&&" << endl;
    // 	for (auto& op : sim_group) {
    // 	  cout << "Diam = " << op.tool_diameter << ", Speed = " << op.spindle_speed << ", Feed = " << op.feedrate << ", DOC = " << op.cut_depth << ", SFM = " << op.SFM() << endl;
    // 	  cout << ", material removed = " << op.material_removed;
    // 	  cout << ", file = " << op.file_name << endl;
    // 	}
    //   }
    // }

    


  // }

  // vector<operation_params> cached = likely_rough_ops;

  // auto voted_op_score =
  //   [&cached](const operation_params& v) {
  //   double score = 0.0;

  //   for (auto& op : cached) {
  //     double sfm_diff = fabs(op.SFM() - v.SFM());
  //     double sfm_v = v.SFM();

  //     if (sfm_diff <= 200.0) {
  // 	double mrr_diff = v.average_MRR() - op.average_MRR();

  // 	// NOTE: Add sfm weight later
  // 	score += mrr_diff;
  //     }
  //   }

  //   return score;
  // };
  // sort_lt(likely_rough_ops, voted_op_score);

  // for (auto& op : likely_rough_ops) {
  //   cout << "----------------------------------------------" << endl;
  //   cout << op << endl;
  //   cout << "SCORE = " << voted_op_score(op) << endl;
  // }

  // return 0;

  // // double num_large_mrrs = count_if(mrrs.begin(), mrrs.end(),
  // // 				   [](double mrr) { return mrr > 5.0; });;
  // // cout << "# files w/ MRR > 10 in^3/min: " << num_large_mrrs << endl;

  // vector<op_replacement> replacements;
  // for (auto& g : grouped) {
  //   for (unsigned i = 0; i < g.size(); i++) {
  //     for (unsigned j = i + 1; j < g.size(); j++) {
  // 	replacements.push_back({g[i], g[j]});
  //     }
  //   }
  // }

  // cout << "# of replacements = " << replacements.size() << endl;

  // delete_if(replacements, []
  // 	    (const op_replacement& l) {
  // 	      return fabs(l.better.SFM() - l.worse.SFM()) > 200.0;
  // 	    });

  // cout << "# of replacements with SFM diff < 200.0 = " << replacements.size() << endl;

  // auto replacement_rank =
  //   [](const op_replacement& l, const op_replacement& r) {
  //   return (l.better.average_MRR() - l.worse.average_MRR()) <
  //   (r.better.average_MRR() - r.worse.average_MRR());
  // };

  // sort(begin(replacements), end(replacements), replacement_rank);

  // cout << endl << "ALL REPLACEMENTS RANKED" << endl;
  // for (auto& rep : replacements) {
  //   cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  //   cout << "========== WORSE ==========" << endl;
  //   cout << rep.worse << endl;
  //   cout << "========== BETTER ==========" << endl;
  //   cout << rep.better << endl;
  //   cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << endl;
  // }

  // time(&end_time);
  // double seconds = difftime(end_time, start_time);
  // cout << "Total time to process all .NCF files: " << seconds << " seconds" << endl;
  

  // cout << "mrrs.size() = " << mrrs.size() << endl;
  //print_histogram(mrrs);
}

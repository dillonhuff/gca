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
#include "gcode/visual_debug.h"
#include "geometry/box.h"
#include "geometry/vtk_debug.h"
#include "simulators/region.h"
#include "simulators/sim_mill.h"
#include "simulators/simulate_operations.h"
#include "synthesis/visual_debug.h"
#include "gcode/cut.h"
#include "gcode/safe_move.h"
#include "gcode/circular_helix_cut.h"
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

bool program_in_GCA_travel(const std::vector<std::vector<cut*> >& paths) {
  double emco_f1_x_travel = 8;
  double emco_f1_y_travel = 6;
  double emco_f1_z_travel = 8;

  box bound = bound_paths(paths);

  if ((bound.x_len() > emco_f1_x_travel) ||
      (bound.y_len() > emco_f1_y_travel) ||
      (bound.z_len() > emco_f1_z_travel)) {
    cout << "Box" << endl;
    cout << bound << endl;
    cout << "goes beyond emco_f1 bounds" << endl;

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

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

point out_of_bounds_point(const double range) {
  double v = range;
  double x = fRand(0, v);
  v = v - x;
  double y = fRand(0, v);
  v = v - y;
  double z = v;
  return point(x, y, z);
}

int random_int(const int min, const int max) {
  return min + (rand() % (int)(max - min + 1));
}

point random_point(const point l, const point r) {
  box bb = bound_positions({l, r});

  double x = fRand(bb.x_min, bb.x_max);
  double y = fRand(bb.y_min, bb.y_max);
  double z = fRand(bb.z_min, bb.z_max);

  return point(x, y, z);
}

circular_arc* random_circular_arc_to_from(const point start, const point end) {
  DBG_ASSERT(start.z == end.z);
  DBG_ASSERT(false);
}

cut* random_cut_to_from(const point start, const point end) {
  int cut_index = random_int(0, 1);
  if (cut_index == 0) {
    return new (allocate<linear_cut>()) linear_cut(start, end);
  } else if (cut_index == 1) {
    return new (allocate<safe_move>()) safe_move(start, end);
  } else {
    DBG_ASSERT(false);
  }
}

cut* random_cut_starting_at(const point start, const point bound) {
  int cut_index = random_int(0, 3);
  if (cut_index == 0) {

    return new (allocate<linear_cut>()) linear_cut(start, random_point(start, bound));

  } else if (cut_index == 1) {

    return new (allocate<safe_move>()) safe_move(start, random_point(start, bound));
  }
  else if (cut_index == 2) {

    point end = random_point(start, bound);
    end.z = start.z;
    point centroid = 0.5*(start + end);
    point center = centroid;
    direction dir = random_int(0, 1) == 1 ? COUNTERCLOCKWISE : CLOCKWISE;

    return new (allocate<circular_arc>()) circular_arc(start, end, center - start, dir, XY);
    
  } else {
    point end = random_point(start, bound);
    point centroid = 0.5*(start + end);
    point center = centroid;
    direction dir = random_int(0, 1) == 1 ? COUNTERCLOCKWISE : CLOCKWISE;//COUNTERCLOCKWISE;

    return new (allocate<circular_arc>()) circular_helix_cut(start, end, center - start, dir, XY);

  }
}

vector<cut*> random_cuts_to_from(const point start, const point end) {
  int max_cuts = 8;
  int num_cuts = random_int(1, max_cuts);

  point first_point = start;
  //point last_point = first_point;

  vector<cut*> cuts;
  for (int i = 0; i < num_cuts; i++) {

    cut* c;
    
    if (i == (num_cuts - 1)) {
      c = random_cut_to_from(first_point, end);
    } else {
      c = random_cut_starting_at(first_point, end);
    }

    first_point = c->get_end();
    //last_point = c->get_end();

    cuts.push_back(c);

  }
  

  return cuts;
}

vector<cut*> random_cut_sequence(const point start_pt, const double range) {
  point start = start_pt;
  point mid = out_of_bounds_point(range);
  point end = start_pt;

  vector<cut*> cuts = random_cuts_to_from(start, mid);
  concat(cuts, random_cuts_to_from(mid, end));
  return cuts;
}


bool randomly_mutate(std::vector<std::vector<cut*> >& paths) {
  if (paths.size() == 0) { return false; }

  double r = ((double) rand() / (RAND_MAX));
  if (r < 0.5) {
    cut* last = paths.back().back();
    point last_pt = last->get_end();
    vector<cut*> error_cuts = random_cut_sequence(last_pt, 100);

    concat(paths.back(), error_cuts); //.push_back(error_cut);

    return true;
  }
  return false;
}

void print_case_stats(const std::vector<mutated_test_case>& cases) {
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

void test_mutated_cases_HAAS(const std::string& dir_name) {
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

  print_case_stats(cases);
}

void test_mutated_cases_GCA(const std::string& dir_name) {
  std::vector<mutated_test_case> cases;

  apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

	bool introduced_error =
	  randomly_mutate(paths);

	bool any_travel_errors = program_in_GCA_travel(paths);

	if (!introduced_error) {
	  cases.push_back({paths, false, any_travel_errors});
	} else {
	  cases.push_back({paths, true, any_travel_errors});
	}
      }
    });

  print_case_stats(cases);
}

void test_bounds_cases_HAAS(const std::string& dir_name) {
  std::vector<mutated_test_case> cases;

  apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

	bool introduced_error = false;

	bool any_travel_errors = program_in_HAAS_travel(paths);

	if (!introduced_error) {
	  cases.push_back({paths, false, any_travel_errors});
	} else {
	  cases.push_back({paths, true, any_travel_errors});
	}
      }
    });

  print_case_stats(cases);
}

void test_bounds_cases_GCA(const std::string& dir_name) {
  std::vector<mutated_test_case> cases;

  apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

	bool introduced_error = false;

	bool any_travel_errors = program_in_GCA_travel(paths);

	if (!introduced_error) {
	  cases.push_back({paths, false, any_travel_errors});
	} else {
	  cases.push_back({paths, true, any_travel_errors});
	}
      }
    });

  print_case_stats(cases);
}

vector<operation_params> generate_params(const std::string& dir_name) {
  vector<operation_params> all_params;

  std::vector<pair<string, simulation_log> > files_to_simulation_logs;
  std::vector<pair<string, int> > files_to_unsafe_moves;

  apply_to_gprograms(dir_name, [&all_params, &files_to_unsafe_moves](const vector<block>& p, const string& file_name) {
      vector<vector<cut*>> paths;
      auto r = gcode_to_cuts(p, paths);
      if (r == GCODE_TO_CUTS_SUCCESS) {

  	//bool any_travel_errors = program_in_HAAS_travel(paths);
	
  	// map<int, tool_info> tt = infer_tool_table_GCA(p);

  	// std::vector<operation_range> op_ranges =
  	//   infer_operation_ranges_GCA(p);

  	// auto ops = program_operations_GCA(paths, tt, op_ranges);
  	// concat(all_params, ops);

  	//simulation_log l = simulation_log_GCA(paths, tt, op_ranges);

  	//files_to_simulation_logs.push_back( make_pair(file_name, prog_ops) );

  	//cout << "# of operations = " << l.operation_logs.size() << endl;

  	map<int, tool_info> tt = infer_tool_table_GCA(p);

  	std::vector<operation_range> op_ranges =
  	  infer_operation_ranges_GCA(p);

  	auto prog_ops = program_operations_GCA(paths, tt, op_ranges);

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

  return all_params;

}

std::string student_name(const operation_params& p) {
  string file = p.file_name;
  string start = "/Users/dillon/Documents/PRL-Project-Folders/";
  string suffix = file.substr(start.size());
  //cout << "Suffix = " << suffix << endl;
  size_t loc = suffix.find_first_of("/");
  string student_name = suffix.substr(0, loc);

  //cout << "student name = " << student_name << endl;
  return student_name;
}

bool all_same_student(const std::vector<operation_params>& params) {
  if (params.size() < 2) { return true; }

  string name = student_name(params[0]);
  for (unsigned i = 1; i < params.size(); i++) {
    operation_params op = params[i];

    if (student_name(op) != name) {
      return false;
    }
  }
  
  return true;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: analyze-gcodes <directory path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];

  test_bounds_cases_HAAS(dir_name);

  return 0;

  generate_params(dir_name);

  return 0;

  auto all_ops = read_operation_params_json(dir_name);

  vector<operation_params> likely_rough_ops = all_ops;
  delete_if(likely_rough_ops,
  	    [](const operation_params& op) {
  	      return (op.range.name != "ROUGHING") ||
  		within_eps(op.tool_diameter, 0.0, 0.0001) ||
  		(op.cut_depth < 0.0) ||
		(op.material_removed < 0.3333);
		//(60*op.average_MRR() < 0.3333);
		//(op.material_removed < 0.125); //pow(op.tool_diameter, 3));
  	    });

  cout << "# of likely rough operations = " << likely_rough_ops.size() << endl;
  // for (auto& op : likely_rough_ops) {
  //   cout << op.SFM() << "," << op.average_MRR() << endl;
  // }

  vector<vector<operation_params> > grouped =
    group_by(likely_rough_ops, [](const operation_params& l,
  				  const operation_params& r) {
  	       return within_eps(l.tool_diameter, r.tool_diameter, 0.001) &&
	       (l.tool_end_type == r.tool_end_type) &&
	       within_eps(l.SFM(), r.SFM(), 1.0);
  	     });

  delete_if(grouped, [](const std::vector<operation_params>& group) {
      return group.size() < 2;
    });

  delete_if(grouped, all_same_student);
  
  sort_lt(grouped, [](const std::vector<operation_params>& group) {
      return group.front().tool_diameter;
    });

  for (auto& group : grouped) {
    sort_lt(group, [](const operation_params& op) {
	return op.average_MRR();
      });
  }


  int total_ops = 0;
  for (auto& g : grouped) {
    total_ops += g.size();
  }

  cout << "Same tool and SFM groups = " << grouped.size() << endl;
  cout << "Total operations in      = " << total_ops << endl;

  for (auto& group : grouped) {
    cout << "===================================================" << endl;
    cout << group.front().tool_diameter << " inch " << to_string(group.front().tool_end_type);
    cout << " at " << group.front().SFM() << " SFM " << endl;

    cout << "# of operations = " << group.size() << endl;    

    double worst_mrr = 60*group.front().average_MRR();
    cout << "Worst MRR = " << worst_mrr << endl;
    cout << group.front() << endl;

    double best_mrr = 60*group.back().average_MRR();
    cout << "Best MRR = " << best_mrr << endl;
    //cout << group.back() << endl;
    double avg = 0.0;
    for (auto& g : group) {
      avg += g.average_MRR();
    }

    avg = 60*(avg / group.size());

    cout << "Average MRR = " << avg << endl;

    double pct_diff = ((best_mrr - worst_mrr) / worst_mrr) * 100;

    cout << "Percent improvement over worst = " << pct_diff << endl;
    
    // for (auto& op : group) {
    //   cout << "-------------------------------------------------" << endl;
    //   cout << op << endl;
    // }
  }

  
  // sort(begin(all_ops), end(all_ops), [](const operation_params& l,
  // 					const operation_params& r) {
  // 	 return l.average_MRR() < r.average_MRR();
  //      });
  
  // for (auto& op : all_ops) {
  //   if (op.range.name == "ROUGHING") {
  //     cout << op.file_name << endl;
  //     cout << op.SFM() << "," << 60*op.average_MRR() << endl;
  //   }
  // }


  //write_logs_to_json(files_to_simulation_logs);

  //return 0;

  // return 0;

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

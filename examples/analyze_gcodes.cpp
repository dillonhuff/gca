#include <cassert>
#include <ctime>
#include <future>
#include <map>
#include <numeric>
#include <streambuf>

#include "analysis/fuzzing.h"
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
#include "transformers/clip_transitions.h"
#include "utils/algorithm.h"
#include "utils/grouping.h"
#include "utils/arena_allocator.h"
#include "system/file.h"
#include "system/json.h"

using namespace gca;
using namespace std;


template<typename F>
void apply_to_gprograms(const string& dn, const string& extension, F f) {

  auto func = [&f, extension](const string& dir_name) {
    if (ends_with(dir_name, extension)) {
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

template<typename F>
void apply_to_gprograms(const string& dn, F f) {
  apply_to_gprograms(dn, ".NCF", f);
}

template<typename F>
void apply_to_cura_programs(const string& dn, F f) {
  apply_to_gprograms(dn, ".gcode", f);
}

template<typename F>
void apply_to_router_gprograms(const string& dn, F f) {
  apply_to_gprograms(dn, ".tap", f);
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


	for (auto& op : prog_ops) {
	  cout << op << endl;
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
  	  infer_operation_ranges_HAAS(p);

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

void compute_perf_groups(const std::vector<operation_params>& all_ops) {
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

}

vector<vector<block> > split_layers(const std::vector<block>& program) {
  string layer_prefix = "LAYER:";
  auto not_layer = [&layer_prefix](const block& b) {
    for (auto& token : b) {
      if (token.tp() == LINE_SEMICOLON_COMMENT) {
	
	if (starts_with(token.text, layer_prefix)) {
	  return true;
	}

      }
    }

    return false;
  };

  auto it = begin(program);
  while (it != end(program)) {
    auto r = find(it, end(program), not_f);
    res.push_back(std::vector<I>(it, r.first + 1));
    it = r.second;
  }
  
  return split_by(program, not_layer);
}

vector<block> add_temperature_gradient(const std::vector<block>& program) {
  string layer_prefix = "LAYER:";
  for (auto& blk : program) {
    for (auto& token : blk) {
      if (token.tp() == LINE_SEMICOLON_COMMENT) {
	
	if (starts_with(token.text, layer_prefix)) {
	  int layer_num = stoi(token.text.substr(layer_prefix.size()));
	  cout << "Layer num = " << layer_num << endl;
	}

      }
    }
  }

  return program;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: analyze-gcodes <directory path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];

  clock_t begin = clock();

  cout << "Start time = " << begin << endl;

  auto print_prog = [](const std::vector<block>& p, const std::string& file_name) {
    vector<block> fp = add_temperature_gradient(p);
    cout << fp.size() << endl;
  };

  apply_to_cura_programs(dir_name, print_prog);
  
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;  

  cout << "Elapsed time = " << elapsed_secs << " secs" << endl;

  return 0;
}

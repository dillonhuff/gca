#include "simulators/simulate_operations.h"

#include "simulators/sim_mill.h"
#include "system/file.h"
#include "utils/algorithm.h"

using namespace std;

namespace gca {

  std::string extract_operation_name(const std::string& op) {
    string rough_str(" ROUGHING )");
    if (ends_with(op, rough_str)) {
      return "ROUGHING";
    }

    string surfacing_str(" SURFACING )");
    if (ends_with(op, surfacing_str)) {
      return "SURFACING";
    }

    string holes_str(" HOLES )");
    if (ends_with(op, holes_str)) {
      return "HOLES";
    }

    string contour_str(" CONTOUR )");
    if (ends_with(op, contour_str)) {
      return "CONTOUR";
    }
  
    cout << "UNRECOGNIZED OP STRING = " << op << endl;
    return "UNKNOWN";
  }

  
  // TODO: Move to string utility file
  bool starts_with(const string& value, const string& prefix) {
    if (prefix.size() > value.size()) return false;
    auto res = std::mismatch(prefix.begin(), prefix.end(), value.begin());
    return res.first == prefix.end();
  }

  
  tool_end read_tool_end(std::string& comment) {
    cout << "tool comment = " << comment << endl;

    string r("ROUGH");
    if (starts_with(comment, r)) {
      return ROUGH_ENDMILL;
    }

    string f("FINISH");
    if (starts_with(comment, f)) {
      return FINISH_ENDMILL;
    }

    string b("BALL");
    if (starts_with(comment, b)) {
      return BALL_ENDMILL;
    }

    string d("DRILL");
    if (starts_with(comment, d)) {
      return DRILL_ENDMILL;
    }

    string sd("SPOT DRILL");
    if (starts_with(comment, sd)) {
      return SPOT_DRILL_ENDMILL;
    }
  
    string fc("FACE");
    if (starts_with(comment, fc)) {
      return FACE_ENDMILL;
    }

    string cs("COUNTERSINK");
    if (starts_with(comment, cs)) {
      return COUNTERSINK_ENDMILL;
    }

    string fly("FLY CUTTER");
    if (starts_with(comment, fly)) {
      return FLY_CUTTER_ENDMILL;
    }

    string kc("KEY CUTTER");
    if (starts_with(comment, kc)) {
      return KEY_CUTTER_ENDMILL;
    }
  
    cout << "UNKNOWN TOOL COMMENT = " << comment << endl;
    return FINISH_ENDMILL;
  }

  
  void add_tool_HAAS(map<int, tool_info>& tt, string& comment) {
    string tool_comment_start = "( TOOL ";
    if (starts_with(comment, tool_comment_start)) {
      cout << "Tool comment is " << comment << endl;
      size_t i = -1;
      int tool_no = stoi(comment.substr(tool_comment_start.size()), &i);
      cout << "tool_no = " << tool_no << endl;
      assert(i != -1);
      string rest = comment.substr(tool_comment_start.size() + i);

      cout << "Rest of comment = " << rest << endl;

      size_t j = -1;
      double tool_diameter = stod(rest, &j);
      string tool_comment = rest.substr(j + 1);
      tool_end end = read_tool_end(tool_comment);
    
      cout << "tool diameter = " << tool_diameter << endl;
      tool_info tf{end, tool_diameter};
      tt[tool_no] = tf;
    }
  }

  // void add_tool_GCA(map<int, tool_info>& tt, string& comment) {
  //   string tool_comment_start = "(*** TOOL DIAMETER = ";
  //   if (starts_with(comment, tool_comment_start)) {
  //     cout << "Tool comment is " << comment << endl;
  //     size_t i = -1;
  //     int tool_no = stoi(comment.substr(tool_comment_start.size()), &i);
  //     cout << "tool_no = " << tool_no << endl;
  //     assert(i != -1);
  //     string rest = comment.substr(tool_comment_start.size() + i);

  //     cout << "Rest of comment = " << rest << endl;

  //     size_t j = -1;
  //     double tool_diameter = stod(rest, &j);
  //     string tool_comment = rest.substr(j + 1);
  //     tool_end end = read_tool_end(tool_comment);
    
  //     cout << "tool diameter = " << tool_diameter << endl;
  //     tool_info tf{end, tool_diameter};
  //     tt[tool_no] = tf;
  //   }
  // }

  
  map<int, tool_info> infer_tool_table_HAAS(const vector<block>& p) {
    vector<token> comments;
    for (auto b : p) {
      for (auto t : b) {
	if ((t.ttp == PAREN_COMMENT) ||
	    (t.ttp == BRACKET_COMMENT)) {
	  comments.push_back(t);
	}
      }
    }
    map<int, tool_info> tt;
    for (auto c : comments) {
      add_tool_HAAS(tt, c.text);
    }
    return tt;
  }

  int extract_tool_number_GCA(const std::string& text) {
    string tool_no_comment_start = "(*** TOOL NUMBER = ";

    cout << "Tool number comment = " << text << endl;

    DBG_ASSERT(starts_with(text, tool_no_comment_start));

    size_t i = -1;
    int tool_no = stoi(text.substr(tool_no_comment_start.size()), &i);
    cout << "tool_no = " << tool_no << endl;
    DBG_ASSERT(i != -1);

    return tool_no;
  }

  map<int, tool_info> infer_tool_table_GCA(const vector<block>& p) {
    vector<token> comments;
    for (auto b : p) {
      for (auto t : b) {
	if ((t.ttp == PAREN_COMMENT) ||
	    (t.ttp == BRACKET_COMMENT)) {
	  comments.push_back(t);
	}
      }
    }
    map<int, tool_info> tt;
    //for (auto c : comments) {
    for (unsigned cnum = 0; cnum < comments.size(); cnum++) {
      auto c = comments[cnum];

      string comment = c.text;

      string tool_comment_start = "(*** TOOL DIAMETER = ";
      if (starts_with(comment, tool_comment_start)) {
	cout << "Tool comment is " << comment << endl;
	size_t i = -1;
	double tool_diameter = stod(comment.substr(tool_comment_start.size()), &i);
	cout << "tool_diameter = " << tool_diameter << endl;
	DBG_ASSERT(i != -1);

	unsigned num_comment_ind = cnum + 2;
	token tool_no_comment = comments[num_comment_ind];

	int tool_no = extract_tool_number_GCA(tool_no_comment.text);
	// string rest = comment.substr(tool_comment_start.size() + i);

	// cout << "Rest of comment = " << rest << endl;

	// size_t j = -1;
	// double tool_diameter = stod(rest, &j);
	// string tool_comment = rest.substr(j + 1);
	// tool_end end = read_tool_end(tool_comment);
    
	// cout << "tool diameter = " << tool_diameter << endl;
	tool_info tf{ROUGH_ENDMILL, tool_diameter};
	tt[tool_no] = tf;
      }
    }
    return tt;
  }


  boost::optional<double> infer_program_length_feet(const vector<block>& p) {
    vector<token> comments;
    for (auto b : p) {
      for (auto t : b) {
	if ((t.ttp == PAREN_COMMENT) ||
	    (t.ttp == BRACKET_COMMENT)) {
	  comments.push_back(t);
	}
      }
    }

    for (auto c : comments) {
      auto comment = c.text;

      string len_comment_start = "( FILE LENGTH ";
      string len_comment_end = " FEET )";
      if (starts_with(comment, len_comment_start) &&
	  ends_with(comment, len_comment_end)) {

	cout << "Length comment is " << comment << endl;

	// size_t i = -1;
	// int tool_no = stoi(comment.substr(tool_comment_start.size()), &i);
	// cout << "tool_no = " << tool_no << endl;
	// DBG_ASSERT(i != -1);
      
	double len = stod(comment.substr(len_comment_start.size()));

	cout << "length = " << len << endl;

	//string rest = comment.substr(len_comment_start.size() + i);

	//double tool_diameter = stod(rest);

	//cout << "tool diameter = " << tool_diameter << endl;

	return len;
      }

    }

    return boost::none;
  }


  std::vector<operation_range> infer_operation_ranges_HAAS(const vector<block>& p) {
    vector<token> comments;
    string op_str("( OPERATION ");

    for (auto b : p) {
      for (auto t : b) {
	if (((t.ttp == PAREN_COMMENT) ||
	     (t.ttp == BRACKET_COMMENT)) &&
	    starts_with(t.text, op_str)) {
	  comments.push_back(t);
	}
      }
    }

    if (comments.size() < 1) {
      return {};
    }

    vector<operation_range> op_ranges;
    string first_op_name = extract_operation_name(comments.front().text);

    operation_range active{first_op_name, comments.front().line_no};

    op_ranges.push_back(active);

    for (unsigned i = 1; i < comments.size(); i++) {
      token next_op_comment = comments[i];
      string op_name = extract_operation_name(next_op_comment.text);
      operation_range active{op_name, next_op_comment.line_no};

      op_ranges.back().end_line = next_op_comment.line_no;

      op_ranges.push_back(active);
    
    }

    int last_line_no = p.back().back().line_no;
    cout << "last line number = " << last_line_no << endl;
  
    op_ranges.back().end_line = last_line_no;

    cout << "# of op ranges = " << op_ranges.size() << endl;
    for (auto& op : op_ranges) {
      cout << op << endl;
    }

    return op_ranges;
  }

  std::vector<operation_range> infer_operation_ranges_GCA(const vector<block>& p) {
    vector<token> comments;
    vector<token> tool_no_comments;

    string op_str("(*** OPERATION = ");
    string tool_no_str("(*** TOOL NUMBER = ");

    for (auto b : p) {
      for (auto t : b) {
	if (((t.ttp == PAREN_COMMENT) ||
	     (t.ttp == BRACKET_COMMENT))) {
	  if (starts_with(t.text, op_str)) {
	    comments.push_back(t);
	  }

	  if (starts_with(t.text, tool_no_str)) {
	    tool_no_comments.push_back(t);
	  }
	}
      }
    }

    DBG_ASSERT(tool_no_comments.size() == comments.size());

    if (comments.size() < 1) {
      return {};
    }

    vector<operation_range> op_ranges;
    string first_op_name = extract_operation_name(comments.front().text);
    int tool_no = extract_tool_number_GCA(tool_no_comments.front().text);

    operation_range active{first_op_name, comments.front().line_no, -1, tool_no};

    op_ranges.push_back(active);

    for (unsigned i = 1; i < comments.size(); i++) {
      token next_op_comment = comments[i];
      token next_tool_no_comment = tool_no_comments[i];
      
      string op_name = extract_operation_name(next_op_comment.text);
      int tool_no = extract_tool_number_GCA(next_tool_no_comment.text);

      operation_range active{op_name, next_op_comment.line_no, -1, tool_no};

      op_ranges.back().end_line = next_op_comment.line_no;

      op_ranges.push_back(active);
    
    }

    int last_line_no = p.back().back().line_no;
    cout << "last line number = " << last_line_no << endl;
  
    op_ranges.back().end_line = last_line_no;

    cout << "# of op ranges = " << op_ranges.size() << endl;
    for (auto& op : op_ranges) {
      cout << op << endl;
    }

    return op_ranges;
  }

  std::string to_string(const tool_end l) {
    switch (l) {

    case ROUGH_ENDMILL:
      return "ROUGH_ENDMILL";

    case BALL_ENDMILL:
      return "BALL_ENDMILL";

    case FINISH_ENDMILL:
      return "FINISH_ENDMILL";

    case DRILL_ENDMILL:
      return "DRILL";

    case FACE_ENDMILL:
      return "FACE";

    case SPOT_DRILL_ENDMILL:
      return "SPOT_DRILL";

    case COUNTERSINK_ENDMILL:
      return "COUNTERSINK";

    case KEY_CUTTER_ENDMILL:
      return "KEY_CUTTER";

    case FLY_CUTTER_ENDMILL:
      return "FLY_CUTTER";

    default:
      DBG_ASSERT(false);
    }
  }

  std::ostream& operator<<(std::ostream& out, const tool_end l) {
    out << to_string(l);

    return out;
  }

  std::ostream& operator<<(std::ostream& out, const operation_range& op_range) {
    out << op_range.name << "TOOL NO: " << op_range.tool_number << ", " << " START: " << op_range.start_line << ", END: " << op_range.end_line;
    return out;
  }

  std::ostream& operator<<(std::ostream& out, const operation_params& op) {

    double cl_2_flute = chip_load(op.spindle_speed, op.feedrate, 2);
    double cl_4_flute = chip_load(op.spindle_speed, op.feedrate, 4);
    double cl_6_flute = chip_load(op.spindle_speed, op.feedrate, 6);

    out << "File name = " << op.file_name << endl;
    out << "Operation range = " << op.range << endl;
    out << "current_tool_no = " << op.current_tool_no << endl;
    out << "Tool diameter = " << op.tool_diameter << endl;
    out << "Tool type     = " << op.tool_end_type << endl << endl;

    out << "cut depth estimate = " << op.cut_depth << endl;
    out << "feedrate estimate = " << op.feedrate << endl;
    out << "spindle speed estimate = " << op.spindle_speed << endl << endl;

    out << "estimated material removed = " << op.material_removed << endl << endl;
    out << "cut length                 = " << op.cut_distance << endl << endl;
    out << "cut time                   = " << op.cut_time << endl << endl;
    out << "estimated average MRR      = " << op.average_MRR() << endl;
    
    out << "implied sfm = " << op.SFM() << endl;

    out << "implied CL for 2 flutes = " << cl_2_flute << endl;
    out << "implied CL for 4 flutes = " << cl_4_flute << endl;
    out << "implied CL for 6 flutes = " << cl_6_flute << endl;

    return out;
  }

  std::vector<operation_params>
  program_operations_HAAS(std::vector<std::vector<cut*> >& paths,
			  map<int, tool_info>& tool_table,
			  const std::vector<operation_range>& op_ranges) {
    if (paths.size() == 0) { return {}; }

    if (op_ranges.size() == 0) { return {}; }

    double max_tool_diameter = 1.5;
    auto r = set_up_region_conservative(paths, max_tool_diameter);

    //vtk_debug_depth_field(r.r);

    auto all_cuts = concat_all(paths);

    unsigned op_ind = 0;
    auto active_op = op_ranges[0];

    vector<pair<operation_range, vector<cut*> > > op_paths;
    vector<cut*> current_path;

    for (auto& cut : all_cuts) {
      if (cut->get_line_number() >= active_op.end_line) {
	op_paths.push_back( make_pair(active_op, current_path) );

	op_ind++;
	active_op = op_ranges[op_ind];

	current_path = {cut};
      } else {
	current_path.push_back(cut);
      }
    
    }

    op_paths.push_back( make_pair(active_op, current_path) );

    DBG_ASSERT(op_paths.size() == op_ranges.size());

    //vtk_debug_cuts(all_cuts);
  
    vector<operation_params> ops;

    for (auto path_op_pair : op_paths) {

      auto path = path_op_pair.second;

      cout << "Looking up tool diameter" << endl;
      auto c_iter = find_if(path.begin(), path.end(),
			    [](const cut* c) { return !c->is_safe_move(); });

      if (c_iter == end(path)) {
	break;
      }

      auto c = *c_iter;

      auto tn = c->settings.active_tool; //path.front()->settings.active_tool;
      if (!(tn->is_ilit())) {
	cout << "ERROR" << endl;
	cout << *c << endl;
	cout << "Active tool = " << *(c->settings.active_tool) << endl;
	assert(false);
      }
      auto tl = static_cast<ilit*>(tn);
      int current_tool_no = tl->v;
      double tool_diameter = tool_table[current_tool_no].tool_diameter;
      tool_end tool_end_type = tool_table[current_tool_no].tool_end_type;
      cylindrical_bit t = (tool_diameter);

      double material_removed = 0.0;
      for (auto c : path) {

	double volume_removed = update_cut(*c, r, t);      
	// Assume no crashes since the program was submitted
	if (!c->is_safe_move()) {
	  material_removed += volume_removed;
	} else {

	  if (!(is_vertical(c) && (c->get_start().z < c->get_end().z))) {
	    double mat_removed_tol = 0.005;
	    if (!within_eps(volume_removed, 0.0, mat_removed_tol)) {
	      cout << "Safe move cuts " << volume_removed << " inches^3 of material!" << endl;
	      cout << "line # = " << c->get_line_number() << endl;
	      cout << *c << endl;
	      material_removed += volume_removed;

	      //DBG_ASSERT(within_eps(volume_removed, 0.0, mat_removed_tol));
	    }
	  }
	}
      }

      double total_length_inches = 0.0;
      double cut_length_inches = 0.0;

      double total_time_seconds = execution_time_seconds(path);
      double cut_time_seconds = 0.0;

      for (auto& c : path) {
	total_length_inches += c->length();

	if (!c->is_safe_move()) {
	  cut_length_inches += c->length();
	  cut_time_seconds += cut_execution_time_seconds(c);
	}
      }

      double cut_depth = estimate_cut_depth_median(path);
      double feedrate = estimate_feedrate_median(path);
      double spindle_speed = estimate_spindle_speed_median(path);
      double sfm = surface_feet_per_minute(spindle_speed, tool_diameter);

      operation_params op{current_tool_no,
	  tool_end_type,
	  tool_diameter,
	  cut_depth,
	  feedrate,
	  spindle_speed,
	  sfm,
	  total_length_inches,
	  cut_length_inches,
	  total_time_seconds,
	  cut_time_seconds,
	  material_removed,
	  "UNKNOWN",
	  path_op_pair.first};

      ops.push_back(op);

      cout << "--------------------------------------------------------" << endl;
      cout << op << endl;
      cout << "--------------------------------------------------------" << endl;
    
    }

    return ops;
  }

  bool same_grid_cell(const grid_update l,
		      const grid_update r) {
    return l.cell == r.cell; //return (l.x_ind == r.x_ind) && (l.y_ind == r.y_ind);
  }

  void sum_updates(const vector<grid_update>& new_updates,
		   vector<grid_update>& total_updates) {

    // cout << "Total updates = " << total_updates.size() << endl;
    // cout << "New updates = " << new_updates.size() << endl;

    for (auto& new_up : new_updates) {
      bool found_update = false;
      for (auto& total_up : total_updates) {
	if (same_grid_cell(new_up, total_up)) {
	  found_update = true;
	  total_up.height_diff += new_up.height_diff;
	  break;
	}
      }

      if (!found_update) {
	total_updates.push_back(new_up);
      }
    }

  }

  std::vector<grid_update> sum_updates(const std::vector<point_update>& updates) {
    vector<grid_update> total_updates;
    for (auto& update : updates) {
      for (auto& gu : update.grid_updates) {
	total_updates.push_back(gu);
      }
    }

    sort(begin(total_updates), end(total_updates),
	 [](const grid_update l, const grid_update& r) {
	   if (l.cell.x_ind < r.cell.x_ind) { return true; }
	   return l.cell.y_ind < r.cell.y_ind;
      });

    vector<vector<grid_update> > same_location_updates =
      split_by(total_updates, [](const grid_update l, const grid_update r) {
	  return l.cell == r.cell;
	});

    vector<grid_update> summed_updates;
    for (auto& group : same_location_updates) {

      grid_cell c = group.front().cell;
      double total_height = 0.0;
      for (auto& update : group) {
	total_height += update.height_diff;
      }

      summed_updates.push_back({c, total_height});
    }

    return total_updates;
  }

  double max_cut_depth_from_updates(const std::vector<point_update>& updates) {
    if (updates.size() == 0) { return -1; }
    vector<grid_update> total_updates = sum_updates(updates);

    if (total_updates.size() == 0) {
      return -1;
    }
    
    grid_update largest_cut = max_e(total_updates, [](const grid_update& g) {
	return g.height_diff;
      });

    return largest_cut.height_diff;
  }

  double volume_removed_in_update(const double resolution,
				  const point_update& update) {
    double volume_removed = 0.0;
    for (auto& g : update.grid_updates) {
      volume_removed += g.height_diff * resolution*resolution;
    }

    return volume_removed;
  }

  struct cut_simulation_log {
    cut* c;
    std::vector<point_update> updates;
  };

  struct operation_info {
    operation_range range;
    tool_info tool_inf;
  };

  struct operation_log {
    std::vector<cut_simulation_log> cuts;
    operation_info info;
  };

  operation_params
  build_operation_summary(const double sim_resolution,
			  const operation_log& op_log) {

    int current_tool_no = op_log.info.range.tool_number;
    double tool_diameter = op_log.info.tool_inf.tool_diameter;
    tool_end tool_end_type = op_log.info.tool_inf.tool_end_type;
    cylindrical_bit t = (tool_diameter);

    double material_removed = 0.0;

    for (auto op : op_log.cuts) {
      auto c = op.c;
      auto updates = op.updates;

      double volume_removed = 0.0;
      for (auto& update : updates) {
	volume_removed += volume_removed_in_update(sim_resolution, update);
      }

      // Assume no crashes since the program was submitted
      if (!c->is_safe_move()) {
	material_removed += volume_removed;
      } else {

	if (!(is_vertical(c) && (c->get_start().z < c->get_end().z))) {
	  double mat_removed_tol = 0.005;
	  if (!within_eps(volume_removed, 0.0, mat_removed_tol)) {
	    cout << "Safe move cuts " << volume_removed << " inches^3 of material!" << endl;
	    cout << "line # = " << c->get_line_number() << endl;
	    cout << *c << endl;
	    material_removed += volume_removed;

	    //DBG_ASSERT(within_eps(volume_removed, 0.0, mat_removed_tol));
	  }
	}
      }

    }

    vector<cut*> path;
    for (auto& op : op_log.cuts) {
      path.push_back(op.c);
    }

    double total_length_inches = 0.0;
    double cut_length_inches = 0.0;

    double total_time_seconds = execution_time_seconds(path);
    double cut_time_seconds = 0.0;

    for (auto& c : path) {
      total_length_inches += c->length();

      if (!c->is_safe_move()) {
	cut_length_inches += c->length();
	cut_time_seconds += cut_execution_time_seconds(c);
      }
    }

    // vtk_debug_depth_field(r.r);

    double cut_depth = estimate_cut_depth_median(path);
    double feedrate = estimate_feedrate_median(path);
    double spindle_speed = estimate_spindle_speed_median(path);
    double sfm = surface_feet_per_minute(spindle_speed, tool_diameter);

    operation_params op{current_tool_no,
	tool_end_type,
	tool_diameter,
	cut_depth,
	feedrate,
	spindle_speed,
	sfm,
	total_length_inches,
	cut_length_inches,
	total_time_seconds,
	cut_time_seconds,
	material_removed,
	"UNKNOWN",
	op_log.info.range};

    return op;
  }

  std::vector<pair<operation_info, vector<cut*> > >
  segment_operations_GCA(std::vector<std::vector<cut*> >& paths,
			 map<int, tool_info>& tool_table,
			 const std::vector<operation_range>& op_ranges) {
    if (paths.size() == 0) { return {}; }

    if (op_ranges.size() == 0) { return {}; }

    auto all_cuts = concat_all(paths);

    unsigned op_ind = 0;
    auto active_op = op_ranges[0];

    vector<pair<operation_range, vector<cut*> > > op_paths;
    vector<cut*> current_path;

    for (auto& cut : all_cuts) {
      if (cut->get_line_number() >= active_op.end_line) {
	op_paths.push_back( make_pair(active_op, current_path) );

	op_ind++;
	active_op = op_ranges[op_ind];

	current_path = {cut};
      } else {
	current_path.push_back(cut);
      }
    
    }

    op_paths.push_back( make_pair(active_op, current_path) );

    DBG_ASSERT(op_paths.size() == op_ranges.size());

    cout << "# of op ranges = " << op_ranges.size() << endl;

    vector<pair<operation_info, std::vector<cut*> > > op_info_path_pairs;
    for (auto path_op_pair : op_paths) {

      auto path = path_op_pair.second;

      int current_tool_no = path_op_pair.first.tool_number;
      double tool_diameter = tool_table[current_tool_no].tool_diameter;
      tool_end tool_end_type = tool_table[current_tool_no].tool_end_type;
      cylindrical_bit t = (tool_diameter);

      tool_info tool_info{tool_end_type, tool_diameter};
      op_info_path_pairs.push_back( {{path_op_pair.first, tool_info}, path} );
    }

    return op_info_path_pairs;
    
  }

  std::vector<operation_log>
  simulate_operations(class region& r,
		      const std::vector<pair<operation_info, std::vector<cut*> > >& op_paths) {

    if (op_paths.size() == 0) { return {}; }

    //vtk_debug_depth_field(r.r);

    //vtk_debug_cuts(all_cuts);

    vector<operation_log> operation_sim_log;
    for (auto path_op_pair : op_paths) {

      operation_info op_info = path_op_pair.first;
      auto path = path_op_pair.second;

      int current_tool_no = op_info.range.tool_number;
      double tool_diameter = op_info.tool_inf.tool_diameter;
      tool_end tool_end_type = op_info.tool_inf.tool_end_type;
      cylindrical_bit t = (tool_diameter);

      std::vector<cut_simulation_log> cut_updates;
      for (auto c : path) {
	vector<point_update> updates = update_cut_with_logging(*c, r, t);
	cut_updates.push_back({c, updates});
      }

      operation_sim_log.push_back({cut_updates, op_info});

    }

    return operation_sim_log;
  }

  std::vector<operation_params>
  program_operations_GCA(std::vector<std::vector<cut*> >& paths,
			 map<int, tool_info>& tool_table,
			 const std::vector<operation_range>& op_ranges) {

    auto op_paths = segment_operations_GCA(paths, tool_table, op_ranges);

    double max_tool_diameter = 1.5;

    auto r = set_up_region_conservative(paths, max_tool_diameter);

    std::vector<operation_log> operation_sim_log =
      simulate_operations(r, op_paths);

    vector<operation_params> ops;

    for (auto& op_log : operation_sim_log) {
      ops.push_back(build_operation_summary(r.r.resolution, op_log));
    }

    return ops;
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

  
}

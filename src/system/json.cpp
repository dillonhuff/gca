#include "system/json.h"

namespace gca {

  ptree encode_json(const triangle_t t) {
    ptree children;

    ptree v1;
    v1.put("", t.v[0]);
    children.push_back(std::make_pair("", v1));

    ptree v2;
    v2.put("", t.v[1]);
    children.push_back(std::make_pair("", v2));

    ptree v3;
    v3.put("", t.v[2]);
    children.push_back(std::make_pair("", v3));

    return children;
  }

  ptree encode_json(const point p) {
    ptree children;

    ptree v1;
    v1.put("", p.x);
    children.push_back(std::make_pair("", v1));

    ptree v2;
    v2.put("", p.y);
    children.push_back(std::make_pair("", v2));

    ptree v3;
    v3.put("", p.z);
    children.push_back(std::make_pair("", v3));

    return children;
  }

  ptree encode_json(const gcode_program& prog) {
    ptree p;
    stringstream ss;
    ss << prog.blocks;
    p.put("", ss.str());
    return p;
  }

  ptree encode_face_list_json(const triangular_mesh& m) {
    ptree children;
    for (auto i : m.face_indexes()) {
      triangle_t t = m.triangle_vertices(i);
      ptree p = encode_json(t);
      children.push_back(std::make_pair("", p));
    }
    return children;
  }

  ptree encode_json(const triangular_mesh& m) {
    ptree p;
    p.add_child("pts", encode_json(m.vertex_list()));
    p.add_child("faces", encode_face_list_json(m));
    return p;
  }

  ptree encode_json(const vice& v) {
    box main = main_box(v);
    box upper_clamp = upper_clamp_box(v);
    box lower_clamp = lower_clamp_box(v);

    vector<triangular_mesh> vice_boxes;
    vice_boxes.push_back(make_mesh(box_triangles(main), 0.001));
    vice_boxes.push_back(make_mesh(box_triangles(upper_clamp), 0.001));
    vice_boxes.push_back(make_mesh(box_triangles(lower_clamp), 0.001));

    return encode_json(vice_boxes);
  }

  // TODO: Encode toolpaths somehow
  ptree encode_json(const fabrication_setup& prog) {
    ptree p;
    p.add_child("partMesh", encode_json(prog.part_mesh()));
    //p.add_child("gcode", encode_json(prog.prog));
    p.add_child("vice", encode_json(prog.v));
    return p;
  }

  ptree encode_json(const fabrication_plan& plan) {
    ptree p;
    p.add_child("setups", encode_json(plan.steps()));
    return p;
  }

  std::vector<tool> decode_tools_json(const ptree& p) {
    return decode_vector<tool>(p);
  }

  fixtures decode_fixtures_json(const ptree& p) {
    vice v = decode_json<vice>(p.get_child("vice"));
    std::vector<plate_height> plates =
      decode_vector<plate_height>(p.get_child("base_plates"));
    return fixtures(v, plates);
  }

  workpiece decode_workpiece_json(const ptree& p) {
    point x = decode_json<point>(p.get_child("x"));
    point y = decode_json<point>(p.get_child("y"));
    point z = decode_json<point>(p.get_child("z"));
    material m = decode_json<material>(p.get_child("workpiece_material"));
    return workpiece(x, y, z, m);
  }

  plan_inputs parse_inputs_json(const std::string& s) {
    ifstream ins(s);
    ptree inputs;
    read_json(ins, inputs);

    std::vector<tool> tools = decode_tools_json(inputs.get_child("tools"));
    DBG_ASSERT(tools.size() > 0);
    fixtures fixes = decode_fixtures_json(inputs.get_child("fixtures"));
    workpiece workpiece_dims = decode_workpiece_json(inputs.get_child("workpiece"));

    return plan_inputs(tools, workpiece_dims, fixes);
  }


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
      // Fix this layout encoding problem once simulator refactor is complete
      point_update all{start, 0, sum};

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

  
}

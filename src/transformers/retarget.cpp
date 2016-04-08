#include "transformers/retarget.h"

namespace gca {

  void sanity_check_height_comp(const vector<cut*>& path) {
    auto lc_setting = path.front()->settings.tool_height_comp;
    for (auto c : path) {
      assert(c->settings.tool_height_comp == lc_setting);
    }
  }

  void sanity_check_toolpath(const vector<cut*>& path) {
    assert(path.size() > 0);
    for (auto c : path) {
      assert(!c->settings.active_tool->is_omitted());
    }
    int tn = get_active_tool_no(path);
    for (auto c : path) {
      value* tl = c->settings.active_tool;
      assert(tl->is_ilit());
      ilit* til = static_cast<ilit*>(tl);
      assert(til->v == tn);
    }
    sanity_check_height_comp(path);
  }

  int select_new_tool(const vector<cut*>& path,
		      const tool_table& old_tools,
		      const tool_table& new_tools) {
    int current = get_active_tool_no(path);
    assert(old_tools.find(current) != end(old_tools));
    assert(new_tools.find(current) != end(new_tools));
    return current;
  }

  vector<cut*> retarget_toolpath(const vector<cut*>& path,
				 tool_table& old_tools,
				 tool_table& new_tools) {
    sanity_check_toolpath(path);
    auto old_tool = old_tools[get_active_tool_no(path)];
    auto t = select_new_tool(path, old_tools, new_tools);
    auto height_comp_setting = path.front()->settings.tool_height_comp;
    vector<cut*> cuts;
    for (auto c : path) {
      auto r = c->copy();
      r->settings.active_tool = ilit::make(t);
      if (height_comp_setting == TOOL_HEIGHT_COMP_NEGATIVE) {
	r = r->shift(point(0, 0, old_tool.length));
      }
      cuts.push_back(r);
    }
    return cuts;
  }

  vector<block> generate_gcode(const vector<cut*>& path) {
    cut_params params;
    params.target_machine = EMCO_F1;
    return cuts_to_gcode(path, params);
  }

  vector<vector<cut*>> haas_to_minimill(const vector<vector<cut*>> & p,
					tool_table& old_tools,
					tool_table& new_tools) {
    vector<vector<cut*>> res_paths;
    for (auto path : p) {
      res_paths.push_back(retarget_toolpath(path, old_tools, new_tools));
    }
    return res_paths;
  }


  vector<vector<block>> haas_to_minimill(const vector<block>& p,
					 tool_table& old_tools,
					 tool_table& new_tools) {
    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(p, paths);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    auto res_paths = haas_to_minimill(paths, old_tools, new_tools);
    vector<vector<block>> result_programs;
    for (auto path : res_paths) {
      result_programs.push_back(generate_gcode(path));
    }
    return result_programs;
  }

}

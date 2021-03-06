#include "transformers/retarget.h"
#include "utils/check.h"

namespace gca {

  void sanity_check_spindle_speed(const vector<cut*>& path) {
    auto ss = get_spindle_speed(path);
    for (auto c : path) {
      auto css = c->settings.spindle_speed;
      DBG_ASSERT(css->is_lit());
      lit* cssl = static_cast<lit*>(css);
      DBG_ASSERT(within_eps(cssl->v, ss));
    }
  }
  
  void sanity_check_height_comp(const vector<cut*>& path) {
    auto lc_setting = path.front()->settings.tool_height_comp;
    for (auto c : path) {
      DBG_ASSERT(c->settings.tool_height_comp == lc_setting);
    }
  }

  void sanity_check_toolpath(const vector<cut*>& path) {
    DBG_ASSERT(path.size() > 0);
    for (auto c : path) {
      DBG_ASSERT(!c->settings.active_tool->is_omitted());
    }
    int tn = get_active_tool_no(path);
    for (auto c : path) {
      value* tl = c->settings.active_tool;
      DBG_ASSERT(tl->is_ilit());
      ilit* til = static_cast<ilit*>(tl);
      DBG_ASSERT(til->v == tn);
    }
    sanity_check_height_comp(path);
    sanity_check_spindle_speed(path);
  }

  int select_new_tool(const vector<cut*>& path,
		      const tool_table& old_tools,
		      const tool_table& new_tools) {
    int current = get_active_tool_no(path);
    DBG_ASSERT(old_tools.find(current) != end(old_tools));
    DBG_ASSERT(new_tools.find(current) != end(new_tools));
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
	r->settings.tool_height_comp = TOOL_HEIGHT_COMP_OFF;
      } else if (height_comp_setting == TOOL_HEIGHT_COMP_POSITIVE) {
	cout << "ERROR: Positive tool height compensation is not supported" << endl;
	DBG_ASSERT(false);
      }
      if (r->settings.active_tool->is_omitted()) {
	cout << "ERROR IN RETARGET TOOLPATH" << endl;
	cout << *c << endl;
	cout << *r << endl;
	DBG_ASSERT(false);
      }
      if (r->settings.spindle_speed->is_omitted()) {
	cout << "ERROR IN RETARGET TOOLPATH" << endl;
	cout << *c << endl;
	cout << *r << endl;
	DBG_ASSERT(false);
      }

      cuts.push_back(r);
    }
    sanity_check_toolpath(cuts);
    return cuts;
  }

  vector<block> generate_gcode(const vector<cut*>& path) {
    sanity_check_toolpath(path);
    cut_params params;
    params.target_machine = EMCO_F1;
    return cuts_to_gcode_no_transitions(path, params);
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
    DBG_ASSERT(r == GCODE_TO_CUTS_SUCCESS);
    auto res_paths = haas_to_minimill(paths, old_tools, new_tools);
    vector<vector<block>> result_programs;
    for (auto path : res_paths) {
      result_programs.push_back(generate_gcode(path));
    }
    return result_programs;
  }

}

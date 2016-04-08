#include "transformers/retarget.h"

namespace gca {

  vector<cut*> retarget_toolpath(const vector<cut*>& path) {
    vector<cut*> cuts = path;
    return cuts;
  }

  vector<block> generate_gcode(const vector<cut*>& path) {
    cut_params params;
    params.target_machine = EMCO_F1;
    return cuts_to_gcode(path, params);
  }

  vector<vector<cut*>> haas_to_minimill(const vector<vector<cut*>> & p,
					const tool_table& old_tools,
					const tool_table& new_tools) {
    vector<vector<cut*>> res_paths;
    for (auto path : p) {
      res_paths.push_back(retarget_toolpath(path));
    }
    return res_paths;
  }


  vector<vector<block>> haas_to_minimill(const vector<block>& p,
					 const tool_table& old_tools,
					 const tool_table& new_tools) {
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

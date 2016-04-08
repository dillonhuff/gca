#include "transformers/retarget.h"

namespace gca {

  vector<cut*> retarget_toolpath(vector<cut*>& path) {
    vector<cut*> cuts;
    return cuts;
  }

  vector<block> generate_gcode(vector<cut*>& path) {
    cut_params params;
    params.target_machine = EMCO_F1;
    return cuts_to_gcode(path, params);
  }

  vector<vector<block>> haas_to_minimill(vector<block>& p) {
    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(p, paths);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    vector<vector<block>> result_programs;    
    for (auto path : paths) {
      auto tp = retarget_toolpath(path);
      result_programs.push_back(generate_gcode(tp));
    }
    return result_programs;
  }

}

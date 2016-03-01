#include "analysis/gcode_to_cuts.h"
#include "core/context.h"
#include "synthesis/shapes_to_gcode.h"
#include "transformers/scale_xyz.h"

namespace gca {

  gprog* scale_xyz(double sf, gprog& p) {
    gcode_settings s;
    s.initial_coord_orient = GCA_ABSOLUTE;
    s.initial_pos = point(0, 0, 0);
    s.initial_tool = DRAG_KNIFE;
    vector<cut*> cuts = gcode_to_cuts(p, s);
    cut_params c;
    c.target_machine = PROBOTIX_V90_MK2_VFD;
    vector<cut*> scaled_cuts;
    for (vector<cut*>::iterator it = cuts.begin(); it != cuts.end(); ++it) {
      scaled_cuts.push_back((*it)->scale(sf));
    }
    return gcode_for_cuts(scaled_cuts, c);
  }
  
}

#include "analysis/gcode_to_cuts.h"
#include "synthesis/shapes_to_gcode.h"
#include "transformers/shift_xyz.h"

namespace gca {

  gprog* shift_xyz(double x_s, double y_s, double z_s, gprog& p) {
    point sf(x_s, y_s, z_s);
    gcode_settings s;
    s.initial_coord_orient = GCA_ABSOLUTE;
    s.initial_pos = point(0, 0, 0);
    s.initial_tool = DRAG_KNIFE;
    vector<cut*> cuts = gcode_to_cuts(p, s);
    cut_params c;
    c.target_machine = PROBOTIX_V90_MK2_VFD;
    vector<cut*> scaled_cuts;
    for (auto ct : cuts) {
      scaled_cuts.push_back(ct->shift(sf));
    }
    return gcode_for_cuts(scaled_cuts, c);
  }
  
}


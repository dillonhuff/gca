#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  shape_layout read_dxf(const char* file, bool log) {
    dxf_reader* listener = new (allocate<dxf_reader>()) dxf_reader(log);
    DL_Dxf* dxf = new (allocate<DL_Dxf>()) DL_Dxf();
    if (!dxf->in(file, listener)) {
      std::cerr << file << " could not be opened.\n";
      assert(false);
    }
    shape_layout shapes_to_cut(listener->cuts,
			       listener->hole_punches,
			       listener->splines);
    return shapes_to_cut;
  }

  gprog* dxf_to_gcode(const char* file, cut_params params) {
    shape_layout shapes_to_cut = read_dxf(file, false);
    return shape_layout_to_gcode(shapes_to_cut, params);
  }

}

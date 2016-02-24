#include "synthesis/dxf_reader.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  shape_layout read_dxf(const char* file, bool log) {
    dxf_reader* listener = new dxf_reader(log);
    DL_Dxf* dxf = new DL_Dxf();
    if (!dxf->in(file, listener)) {
      std::cerr << file << " could not be opened.\n";
      assert(false);
    }
    delete dxf;
    shape_layout shapes_to_cut(listener->cuts,
			       listener->hole_punches,
			       listener->splines);
    delete listener;
    return shapes_to_cut;
  }

  gprog* dxf_to_gcode(const char* file, cut_params params, point shift) {
    shape_layout shapes_to_cut = read_dxf(file);
    return shape_layout_to_gcode(shapes_to_cut, params, shift);
  }

}

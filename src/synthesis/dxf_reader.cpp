#include "synthesis/dxf_reader.h"
#include "synthesis/dxf_to_gcode.h"

namespace gca {

  shape_layout read_dxf(const char* file) {
    dxf_reader* listener = new dxf_reader();
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

  gprog* dxf_to_gcode(char* file, cut_params params) {
    shape_layout shapes_to_cut = read_dxf(file);
    return shape_layout_to_gcode(shapes_to_cut, params);
  }



}

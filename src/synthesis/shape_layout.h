#ifndef GCA_SHAPE_READER_H
#define GCA_SHAPE_READER_H

#include "backend/cut_params.h"
#include "geometry/b_spline.h"
#include "gcode/cut.h"
#include "gcode/hole_punch.h"

namespace gca {

  class shape_layout {
  public:
    vector<cut*> lines;
    vector<hole_punch*> holes;
    vector<b_spline*> splines;

  shape_layout(const vector<cut*>& linesp,
	       const vector<hole_punch*>& holesp,
	       const vector<b_spline*>& splinesp) :
    lines(linesp), holes(holesp), splines(splinesp) {}
  };

}

#endif

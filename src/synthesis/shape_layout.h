#ifndef GCA_SHAPE_READER_H
#define GCA_SHAPE_READER_H

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

  class cut_params {
  public:
    double safe_height;
    double material_depth;
    double cut_depth;
    double push_depth;
    point start_loc;
    point start_orient;
  };

}

#endif

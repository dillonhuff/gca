#ifndef GCA_BOX_H
#define GCA_BOX_H

#include <cassert>
#include <iostream>
#include <vector>

#include "geometry/point.h"

using namespace std;

namespace gca {

  struct box {
    double x_min, x_max, y_min, y_max, z_min, z_max;
    box(double x_minp, double x_maxp,
	double y_minp, double y_maxp,
	double z_minp, double z_maxp) :
      x_min(x_minp), x_max(x_maxp),
      y_min(y_minp), y_max(y_maxp),
      z_min(z_minp), z_max(z_maxp) {
      if (x_min > x_max)
	{ cout << x_min << " > " << x_max << endl; assert(false); } 
      if (y_min > y_max)
	{ cout << y_min << " > " << y_max << endl; assert(false); } 
      if (z_min > z_max)
	{ cout << z_min << " > " << z_max << endl; assert(false); } 
    }
  };

  ostream& operator<<(ostream& out, const box& b);

  bool overlap(const box l, const box r);

  box bound_positions(const vector<point>& pts);
  box bound_boxes(const vector<box>& boxes);
  bool fits_inside(const box& inner, const box& outer);
}

#endif

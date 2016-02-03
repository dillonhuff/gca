#ifndef GCA_B_SPLINE_H
#define GCA_B_SPLINE_H

#include <vector>

#include "geometry/point.h"

using namespace std;

namespace gca {

  class b_spline {
  protected:
    int degree;
    vector<point> control_points;
    vector<double> knots;

    double basis(int i, int k, double x) {
      if (k == 0) {
	double ti = knots[i];
	double tip1 = knots[i+1];
	if (ti <= x && x <= tip1) {
	  return 1;
	}
	return 0;
      }
      double bl_c = basis(i, k - 1, x);
      double bl = bl_c == 0.0 ? 0.0 : (x - knots[i]) / (knots[i+k] - knots[i]);
      double br_c = basis(i + 1, k - 1, x);
      double br = br_c == 0.0 ? 0.0 : (knots[i + k + 1] - x) / (knots[i + k + 1] - knots[i + 1]);
      return bl + br;
    }

  public:
    b_spline(int degreep,
	     const vector<point>& control_pointsp,
	     const vector<double>& knotsp) :
    degree(degreep), control_points(control_pointsp), knots(knotsp) {}

    point eval(double v) {
      point res = point(0, 0, 0);
      for (unsigned i = 0; i < control_points.size(); i++) {
	res = res + basis(i, degree, v) * control_points[i];
      }
      return res;
    }
  };
}

#endif

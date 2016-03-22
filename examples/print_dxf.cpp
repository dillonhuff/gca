#include <cassert>
#include <dirent.h>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "core/parser.h"
#include "geometry/box.h"
#include "geometry/polyline.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/dxf_reader.h"

using namespace gca;

pair<double, double> x_minmax(const vector<polyline>& ps) {
  vector<double> x_values;
  for (auto p : ps) {
    for (auto pt : p) {
      x_values.push_back(pt.x);
    }
  }
  auto p = minmax_element(x_values.begin(), x_values.end());
  return pair<double, double>(*(p.first), *(p.second));
}

pair<double, double> y_minmax(const vector<polyline>& ps) {
  vector<double> y_values;
  for (auto p : ps) {
    for (auto pt : p) {
      y_values.push_back(pt.y);
    }
  }
  auto p = minmax_element(y_values.begin(), y_values.end());
  return pair<double, double>(*(p.first), *(p.second));
}

box polylines_bounding_box(const vector<polyline>& ps) {
  auto xminmax = x_minmax(ps);
  auto yminmax = y_minmax(ps);
  double x_min = xminmax.first;
  double x_max = xminmax.second;
  double y_min = yminmax.first;
  double y_max = yminmax.second;
  box b(x_min, x_max,
	y_min, y_max);
  return b;
}

pair<double, double> box_scale_factors(const box dest, const box source) {
  double src_x_diff = source.x_max - source.x_min;
  double src_y_diff = source.y_max - source.y_min;
  double dst_x_diff = dest.x_max - dest.x_min;
  double dst_y_diff = dest.y_max - dest.y_min;
  return pair<double, double>(dst_x_diff / src_x_diff,
			      dst_y_diff / src_y_diff);
}

double proportional_scale_factor(const box dest, const box source) {
  auto p = box_scale_factors(dest, source);
  return min(p.first, p.second);
}

vector<polyline> fit_in_box(const box b,
			    const vector<polyline>& polys) {
  box poly_bounds = polylines_bounding_box(polys);
  auto sf = proportional_scale_factor(b, poly_bounds);
  vector<polyline> scaled;
  auto scale_pt = [sf](const point p)
    { return point(sf * p.x, sf * p.y, p.z); };
  for (auto p : polys) {
    scaled.push_back(apply_to_points(p, scale_pt));
  }
  box scaled_bounds = polylines_bounding_box(scaled);
  double x_shift = b.x_min - scaled_bounds.x_min;
  double y_shift = b.y_min - scaled_bounds.y_min;
  vector<polyline> shifted;
  auto shift_pt = [x_shift, y_shift](const point p)
    { return point(p.x + x_shift, p.y + y_shift, p.z); };
  for (auto p : scaled) {
    shifted.push_back(apply_to_points(p, shift_pt));
  }
  return shifted;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-dxf <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  cut_params params;
  params.default_feedrate = 10;
  params.set_default_feedrate = true;
  params.material_depth = 0.075;
  params.push_depth = 0.00;
  params.cut_depth = 0.05;
  params.safe_height = 0.45;
  params.machine_z_zero = -1.904;
  params.start_loc = point(5, 5, 0);
  params.start_orient = point(0, -1, 0);
  params.tools = DRAG_KNIFE_ONLY;
  params.target_machine = PROBOTIX_V90_MK2_VFD;
  
  auto l = read_dxf(argv[1]);
  auto sf = [](const vector<polyline>& ps)
    { return fit_in_box(box(9, 12.8, 6.2, 8.1), ps); };
  vector<cut*> scuts = shape_cuts_p(l, params, sf);
  string s = cuts_to_gcode_string(scuts, params);
  auto p = parse_gprog(s);
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  p->print_nc_output(cout);

  int num_warns = check_bounds(p, GCA_ABSOLUTE,
  			       0.5, 17,			       
  			       1.2, 13.2,
  			       -4.1, -0.05);
  if (num_warns > 0) {
    cout << "Num warnings: " << num_warns << endl;
  }
  return 0;
}

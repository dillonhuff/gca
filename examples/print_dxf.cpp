#include <cassert>
#include <dirent.h>
#include <iostream>

#include "checkers/bounds_checker.h"
#include "core/parser.h"
#include "geometry/polyline.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/dxf_reader.h"

using namespace gca;

struct box {
  double x_min, x_max, y_min, y_max;
  box(double x_minp, double x_maxp, double y_minp, double y_maxp) :
    x_min(x_minp), x_max(x_maxp), y_min(y_minp), y_max(y_maxp) {
    if (x_min > x_max)
      { cout << x_min << " > " << x_max << endl; assert(false); } 
    if (y_min > y_max)
      { cout << y_min << " > " << y_max << endl; assert(false); } 
  }
};

ostream& operator<<(ostream& out, const box& b) {
  out << "X min = " << b.x_min << endl;
  out << "X max = " << b.x_max << endl;
  out << "Y min = " << b.y_min << endl;
  out << "Y max = " << b.y_max << endl;
  return out;
}

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
  //  cout << "X minmax: " << *(xminmax.first) << " " << *(xminmax.second) << endl;
  auto yminmax = y_minmax(ps);
  //  cout << "Y minmax: " << *(yminmax.first) << " " << *(yminmax.second) << endl;
  double x_min = xminmax.first;
  double x_max = xminmax.second;
  double y_min = yminmax.first;
  double y_max = yminmax.second;
  box b(x_min, x_max,
	y_min, y_max);
  //  cout << "polylines_bounding_box done" << endl;
  return b;
}

pair<double, double> box_scale_factors(const box dest, const box source) {
  //  cout << "Dest: " << endl;
  // cout << dest << endl;
  // cout << "source: " << endl;
  // cout << source << endl;
  double src_x_diff = source.x_max - source.x_min;
  double src_y_diff = source.y_max - source.y_min;
  double dst_x_diff = dest.x_max - dest.x_min;
  double dst_y_diff = dest.y_max - dest.y_min;
  // cout << "dst_x_diff = " << dst_x_diff << endl;
  // cout << "src_x_diff = " << src_x_diff << endl;
  // cout << "dst_y_diff = " << dst_y_diff << endl;
  // cout << "src_y_diff = " << src_y_diff << endl;
  return pair<double, double>(dst_x_diff / src_x_diff,
			      dst_y_diff / src_y_diff);
}

double proportional_scale_factor(const box dest, const box source) {
  auto p = box_scale_factors(dest, source);
  // cout << "X scale factor: " << p.first << endl;
  // cout << "Y scale factor: " << p.second << endl;
  return min(p.first, p.second);
}

vector<polyline> fit_in_box(const box b,
			    const vector<polyline>& polys) {
  // cout << "Bounding box: " << endl;
  // cout << b << endl;
  box poly_bounds = polylines_bounding_box(polys);
  // cout << "Bounding box after poly_bounding_box call: " << endl;
  // cout << b << endl;
  auto sf = proportional_scale_factor(poly_bounds, b);
  //  cout << "Scaling factor: " << sf << endl;
  vector<polyline> scaled;
  auto scale_pt = [sf](const point p)
    { return point(sf * p.x, sf * p.y, p.z); };
  for (auto p : polys) {
    scaled.push_back(apply_to_points(p, scale_pt));
  }
  return scaled;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-dxf <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  cut_params params;
  params.default_feedrate = 30;
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
    { return fit_in_box(box(0.5, 17, 1.2, 13.2), ps); };
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

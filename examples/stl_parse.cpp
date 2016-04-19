#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

vector<block> polylines_cuts(const vector<polyline>& pocket_lines,
			     const cut_params params) {
  vector<cut*> cuts;
  for (auto p : pocket_lines) {
    auto cs = polyline_cuts(p);
    cuts.insert(cuts.end(), cs.begin(), cs.end());
  }
  return cuts_to_gcode(cuts, params);
}

polyline extract_part_base_outline(const vector<triangle>& tris) {
  auto triangles = tris;
  delete_if(triangles, [](const triangle& t)
	    { return !within_eps(t.normal, point(0, 0, -1), 1e-2); });
  auto outlines = merge_triangles(triangles);
  assert(outlines.size() == 1);
  auto base_outline = outlines.front();
  vector<point> vertices = base_outline.vertices;
  vertices.push_back(vertices.front());
  return polyline(vertices);
}

// template<typename InputIt>
// pocket_info_2P5D surface_pocket_info(double z_level,
// 				     InputIt surface,
// 				     InputIt below,
// 				     InputIt end) {
//   // auto polygons = merge_triangles(vector<triangle>(surface, end));
//   // //assert(polygons.size() == 1);
//   // vector<point> vertices = polygons.front().vertices;
//   // vertices.push_back(vertices.front());
//   // polyline p(vertices);
//   // return pocket_info_2P5D(p, z_level, p.front().z);
// }

// vector<block> generate_mill_paths(const polyline& outline,
// 				  vector<vector<triangle>>& surfaces,
// 				  double tool_diameter) {
//   stable_sort(begin(surfaces), end(surfaces),
// 	      [](const vector<triangle>& tsl,
// 		 const vector<triangle>& tsr)
// 	      { return tsl.front().v1.z > tsr.front().v1.z; });
//   double start_depth = surfaces.front().front().v1.z;
//   double z_level = start_depth;
//   auto below_level = begin(surfaces);
//   auto above_level = begin(surfaces);
//   vector<polyline> pocket_lines;
//   while (below_level < end(surfaces)) {
//     pocket_info_2P5D sa = surface_pocket_info(z_level,
// 					      below_level,
// 					      above_level,
// 					      end(surfaces));
//     auto pcs = pocket_2P5D_interior(sa, tool_diameter);
//     below_level = find_if(below_level, end(surfaces),
// 			  [z_level](const vector<triangle>& t)
// 			  { return !within_eps(t.front().v1.z, z_level); });
//     pocket_lines.insert(end(pocket_lines), begin(pcs), end(pcs));
//   }
//   double end_depth = outline.front().z;
//   pocket_info_2P5D pocket(outline, start_depth, end_depth);
//   auto pocket_cuts = pocket_2P5D_exterior(pocket);
//   pocket_lines.insert(end(pocket_lines), begin(pocket_cuts), end(pocket_cuts));

//   cut_params params;
//   params.target_machine = EMCO_F1;
//   params.safe_height = start_depth + 0.05;
//   return polylines_cuts(pocket_lines, params);
// }

vector<block> emco_f1_code(const vector<polyline>& pocket_lines,
			   double start_depth) {
  cut_params params;
  params.target_machine = EMCO_F1;
  params.safe_height = start_depth + 0.05;
  return polylines_cuts(pocket_lines, params);
}

vector<point> sample_points_2d(const box b, double x_inc, double y_inc, double z_level) {
  vector<point> pts;
  double x = b.x_min;
  while (x < b.x_max) {
    double y = b.y_min;
    while (y < b.y_max) {
      pts.push_back(point(x, y, z_level));
      y += y_inc;
    }
    x += x_inc;
  }
  cout << "# pts = " << pts.size() << endl;
  return pts;
}


box bounding_box(const oriented_polygon& p) {
  return bound_positions(p.vertices);
}

// Make this actual polygon containment
bool contains(const oriented_polygon& g, point p) {
  box b = bounding_box(g);
  if ((b.x_min <= p.x && p.x <= b.x_max) && (b.y_min <= p.y && p.y <= b.y_max)) {
    cout << "Deleting" << endl;
    return true;
  } else {
    return false;
  }
}

// TODO: Fill in
template<typename InputIt>
bool contained_by_any(point p, InputIt l, InputIt r) {
  while (l != r) {
    if (contains(*l, p)) {
      return true;
    }
    ++l;
  }
  return false;
}
template<typename InputIt>
box bounding_box(InputIt s, InputIt e) {
  vector<box> boxes;
  while (s != e) {
    boxes.push_back(bounding_box(*s));
    ++s;
  }
  return bound_boxes(boxes);
}

bool overlaps(line l, const oriented_polygon& p) {
  polyline pl(p.vertices);
  for (auto pll : pl.lines()) {
    if (segment_intersection_2d(l, pll).just)
      { return true; }
  }
  return false;
}

template<typename InputIt>
bool overlaps_any(line l, InputIt s, InputIt e) {
  while (s != e) {
    if (overlaps(l, *s)) {
      return true;
    }
    ++s;
  }
  return false;
}

template<typename InputIt>
vector<polyline> level_roughing(InputIt s, InputIt m, InputIt e, double last_level) {
  box b = bounding_box(m, e);
  cout << "Bounding box" << endl;
  cout << b << endl;
  // Select sample rate from tool_diameter
  auto toolpath_points = sample_points_2d(b, 0.05, 0.05, last_level);
  delete_if(toolpath_points,
	    [s, m](const point p)
	    { return contained_by_any(p, s, m); });
  assert(toolpath_points.size() > 0);
  vector<vector<point>> lpts;
  split_by(toolpath_points, lpts,
	   [s, m](const point l, const point r)
	   { return !overlaps_any(line(l, r), s, m); });
  vector<polyline> lines;
  for (auto ls : lpts) {
    lines.push_back(ls);
  }
  return lines;
}

vector<block> generate_mill_paths(const vector<triangle>& triangles,
				  double tool_diameter) {
  auto polygons = merge_triangles(triangles);
  stable_sort(begin(polygons), end(polygons),
	      [](const oriented_polygon& x,
		 const oriented_polygon& y)
	      { return x.vertices.front().z > y.vertices.front().z; });
  cout << "# of polygons: " << polygons.size() << endl;  
  for (auto p : polygons) {
    cout << "z level = " << p.vertices.front().z << endl;
  }
  double start_depth = polygons.front().vertices.front().z;
  double last_level = start_depth;
  auto below_level = begin(polygons);
  vector<polyline> pocket_lines;
  while (below_level != end(polygons)) {
    auto level_rough = level_roughing(begin(polygons),
				      below_level,
				      end(polygons),
				      last_level);
    pocket_lines.insert(end(pocket_lines), begin(level_rough), end(level_rough));
    below_level = find_if(below_level, end(polygons),
			  [last_level](const oriented_polygon& p)
			  { return !within_eps(p.vertices.front().z, last_level, 0.01); });
    if (below_level != end(polygons)) {
      last_level = (*below_level).vertices.front().z;
    }
  }
  return emco_f1_code(pocket_lines, start_depth);
}

int main(int argc, char* argv[]) {
  arena_allocator a;
  set_system_allocator(&a);

  assert(argc == 2);

  auto info = parse_stl(argv[1]);
  vector<triangle> triangles = info.triangles;
  delete_if(triangles,
	    [](const triangle t)
	    { return !is_upward_facing(t, 0.01); });
  auto bs = generate_mill_paths(triangles, 0.1);

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);
  cout << bs << endl;
}

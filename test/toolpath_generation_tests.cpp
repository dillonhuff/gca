#include "catch.hpp"
#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/rotation.h"
#include "geometry/vtk_utils.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

namespace gca {

  bool overlap_2D(const polyline& lines, const polygon_3& poly) {
    const rotation r = rotate_from_to(poly.normal(), point(0, 0, 1));
    auto line_str = to_boost_linestring(apply(r, lines));
    auto pl = to_boost_poly_2(apply(r, poly));
    return bg::intersects(line_str, pl);
  }

  bool
  overlap_2D(const std::vector<polyline>& lines,
	     const polygon_3& hole) {
    return any_of(begin(lines), end(lines),
		  [hole](const polyline& pl) { return overlap_2D(pl, hole); });
  }

  TEST_CASE("Toolpath compression, nothing to compress") {
    vector<point> pts{point(0, 0, 0), point(1, 0, 0)};
    polyline p(pts);
    REQUIRE(compress_lines(p, 0.01).num_points() == 2);
  }

  TEST_CASE("Toolpath compression, redundant lines") {
    vector<point> pts{point(0, 0, 0),
	point(1, 1, 0),
	point(2, 2, 0),
	point(3, 3, 0)};
    polyline p(pts);
    REQUIRE(compress_lines(p, 0.001).num_points() == 2);
  }

  TEST_CASE("Contour with hole") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<point> safe_ring{point(-3, -3, 0),
	point(3, -3, 0),
	point(3, 3, 0),
	point(-3, 3, 0)};

    vector<point> outer_machine_ring{point(-2, -2, 0),
	point(2, -2, 0),
	point(2, 2, 0),
	point(-2, 2, 0)};

    vector<point> hole{point(-1, -1, 0),
	point(1, -1, 0),
	point(1, 1, 0),
	point(-1, 1, 0)};

    polygon_3 safe_area(safe_ring, {hole});
    polygon_3 machine_area(outer_machine_ring, {hole});

    flat_region r(safe_area, machine_area, 0.1, 0.0, ALUMINUM);

    tool t{1.0 / 8.0, 3.94, 4, HSS, FLAT_NOSE};
    t.set_cut_diameter(1.0 / 8.0);
    t.set_cut_length(1.2);

    t.set_shank_diameter(0.5);
    t.set_shank_length(0.05);

    t.set_holder_diameter(2.5);
    t.set_holder_length(3.5);

    // auto poly_acts = polygon_3_actors(safe_area);
    // visualize_actors(poly_acts);

    // auto int_polys = interior_offset({safe_area}, t.radius());
    // for (auto itp : int_polys) {
    //   concat(poly_acts, polygon_3_actors(itp));
    // }

    // visualize_actors(poly_acts);
    
    std::vector<toolpath> toolpaths = machine_flat_region(r, 1.0, {t});
    polygon_3 offset_hole = exterior_offset(hole, t.radius() - 0.01);

    for (auto& tp : toolpaths) {
      // auto tp_pd = polydata_for_toolpath(tp);
      // auto tp_act = polydata_actor(tp_pd);

      // color white(255, 255, 255);
      // color tp_color = random_color(white);
      // color_polydata(tp_pd, tp_color.red(), tp_color.green(), tp_color.blue());

      // auto poly_acts = polygon_3_actors(offset_hole);
      // concat(poly_acts, polygon_3_actors(hole));
      // poly_acts.push_back(tp_act);
      // visualize_actors(poly_acts);
      REQUIRE(!overlap_2D(tp.lines(), offset_hole));
    }
  }

}

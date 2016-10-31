#include "catch.hpp"
#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/rotation.h"
#include "geometry/vtk_utils.h"
#include "simulators/sim_mill.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

namespace gca {

  bool
  toolpaths_cover_percent_of_area(const flat_region& machine_region,
				  const std::vector<toolpath>& toolpaths,
				  const double required_threshold) {

    DBG_ASSERT(0.0 <= required_threshold);
    DBG_ASSERT(required_threshold <= 100.0);

    std::vector<std::vector<cut*>> all_cuts;
    for (auto& tp : toolpaths) {
      concat(all_cuts, tp.cuts_without_safe_moves());
    }

    toolpath max_diam_t =
      max_e(toolpaths, [](const toolpath& t) { return t.t.cut_diameter(); });

    box b = bound_paths(all_cuts);
    double material_height = machine_region.height();
    class region sim_region = bounding_region(max_diam_t.t.cut_diameter(), b, material_height);

    double volume_cut = 0.0;
    for (auto& tp : toolpaths) {
      cylindrical_bit current_tool(tp.t.cut_diameter());

      for (auto cut_sequence : tp.cuts_without_safe_moves()) {
	volume_cut += simulate_mill(cut_sequence, sim_region, current_tool);
      }
    }

    cout << "Volume cut                         = " << volume_cut << endl;

    double volume_to_cut = area(machine_region.machine_area)*machine_region.height();

    cout << "Volume that was supposed to be cut = " << volume_to_cut << endl;

    double pct_left = (volume_to_cut - volume_cut) / volume_to_cut;

    return within_eps(pct_left, 0.0, 100.0 - required_threshold);
  }

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
    t.set_tool_number(1);


    tool tiny_tool{1.0 / 800.0, 3.94, 4, HSS, FLAT_NOSE};
    tiny_tool.set_cut_diameter(1.0 / 800);
    tiny_tool.set_cut_length(1.2);

    tiny_tool.set_shank_diameter(0.5);
    tiny_tool.set_shank_length(0.05);

    tiny_tool.set_holder_diameter(2.5);
    tiny_tool.set_holder_length(3.5);
    tiny_tool.set_tool_number(2);

    tool huge_tool{1.5, 3.94, 2, HSS, FLAT_NOSE};
    huge_tool.set_cut_diameter(1.5);
    huge_tool.set_cut_length(1.0);

    huge_tool.set_shank_diameter(0.5);
    huge_tool.set_shank_length(0.05);

    huge_tool.set_holder_diameter(2.5);
    huge_tool.set_holder_length(3.5);
    huge_tool.set_tool_number(3);

    SECTION("No overlap with 1/8 inch tool") {
      std::vector<toolpath> toolpaths = machine_flat_region(r, 1.0, {t});
      polygon_3 offset_hole = exterior_offset(hole, t.radius() - 0.01);

      for (auto& tp : toolpaths) {
	REQUIRE(!overlap_2D(tp.lines(), offset_hole));
      }
    }

    SECTION("No use of tiny tool") {
      std::vector<toolpath> toolpaths = machine_flat_region(r, 1.0, {t, tiny_tool});

      for (auto& tp : toolpaths) {
	REQUIRE(tp.tool_number() != 2);
      }
    }

    SECTION("Toolpaths cover most of the region") {
      std::vector<toolpath> toolpaths = machine_flat_region(r, 1.0, {t, huge_tool});

      REQUIRE(toolpaths_cover_percent_of_area(r, toolpaths, 98.0));
    }

  }

  
}

#include "catch.hpp"

#include "backend/feedrate_optimization.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/rotation.h"
#include "geometry/vtk_utils.h"
#include "simulators/sim_mill.h"
#include "synthesis/fabrication_plan.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

namespace gca {

  double
  volume_cut_inside_area(const class region& sim_region,
			 const std::vector<polygon_3>& area_polys,
			 const double original_height) {
    auto poly_2d = to_boost_multipoly_2(area_polys);

    double volume_in_area = 0.0;

    int num_in_region = 0;

    for (int i = 0; i < sim_region.num_x_elems; i++) {
      for (int j = 0; j < sim_region.num_y_elems; j++) {
	double r_x = sim_region.resolution*i;
	double r_y = sim_region.resolution*j;

	point dummy(r_x, r_y, 0.0);
	point converted = sim_region.region_coords_to_machine_coords(dummy);

	bg::model::d2::point_xy<double> conv_pt(converted.x, converted.y);

	if (bg::within(conv_pt, poly_2d)) {
	  num_in_region++;
	  volume_in_area +=
	    sim_region.resolution *
	    sim_region.resolution *
	    (original_height - sim_region.column_height(i, j));
	}
      }
    }

    cout << "# of points in region = " << num_in_region << endl;
    cout << "Volume inside cut area = " << volume_in_area << endl;

    return volume_in_area;
  }

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

    for (auto& tp : toolpaths) {
      cylindrical_bit current_tool(tp.t.cut_diameter());

      for (auto cut_sequence : tp.cuts_without_safe_moves()) {
	simulate_mill(cut_sequence, sim_region, current_tool);
      }
    }

    double volume_cut =
      volume_cut_inside_area(sim_region, machine_region.machine_area, material_height);

    cout << "Volume cut                         = " << volume_cut << endl;

    double volume_to_cut = area(machine_region.machine_area)*machine_region.height();

    cout << "Volume that was supposed to be cut = " << volume_to_cut << endl;

    DBG_ASSERT((volume_cut <= volume_to_cut) ||
	       within_eps(volume_cut, volume_to_cut, 0.5));

    double pct_left = ((volume_to_cut - volume_cut) / volume_to_cut)*100;

    cout << "Percent of volume left = " << pct_left << endl;

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

  TEST_CASE("Island contour generation") {
    arena_allocator a;
    set_system_allocator(&a);

    tool t{1.0 / 8.0, 3.94, 4, HSS, FLAT_NOSE};
    t.set_cut_diameter(1.0 / 2.0);
    t.set_cut_length(1.25);

    t.set_shank_diameter(0.5);
    t.set_shank_length(0.05);

    t.set_holder_diameter(2.5);
    t.set_holder_length(3.5);
    t.set_tool_number(1);

    vector<point> safe_pts{point(-2, -2, 0),
	point(-2, 2, 0),
	point(2, 2, 0),
	point(2, -2, 0)};

    vector<point> block_pts{point(-1, -1, 0),
	point(-1, 1, 0),
	point(1, 1, 0),
	point(1, -1, 0)};

    vector<point> island_pts{point(-0.9, -0.5, 0),
	point(-0.9, 0.5, 0),
	point(0.9, 0.5, 0),
	point(0.9, -0.5, 0)};

    polygon_3 safe = build_clean_polygon_3(safe_pts, {island_pts});
    
    polygon_3 block = build_clean_polygon_3(block_pts, {island_pts});

    flat_region r(safe, block, 0.5, 0.0, ALUMINUM);

    vector<toolpath> toolpaths = machine_flat_region_with_contours(r, 1.0, {t});

    // vector<vtkSmartPointer<vtkActor> > actors = polygon_3_actors(block);
    // for (auto tp : toolpaths) {
    //   actors.push_back(actor_for_toolpath(tp));
    // }

    // visualize_actors(actors);

    REQUIRE(toolpaths_cover_percent_of_area(r, toolpaths, 98.0));
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

    polygon_3 safe_area = build_clean_polygon_3(safe_ring, {hole});
    polygon_3 machine_area = build_clean_polygon_3(outer_machine_ring, {hole});

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
      polygon_3 offset_hole = exterior_offset(build_clean_polygon_3(hole), t.radius() - 0.01);

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

  TEST_CASE("Contour with hole with recess") {
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
	point(-0.1, -1, 0),
	point(-0.1, 0, 0),
	point(0.1, 0, 0),
	point(0.1, -1, 0),
	point(1, -1, 0),
	point(1, 1, 0),
	point(-1, 1, 0)};

    polygon_3 safe_area = build_clean_polygon_3(safe_ring, {hole});
    polygon_3 machine_area = build_clean_polygon_3(outer_machine_ring, {hole});

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
      polygon_3 offset_hole = exterior_offset(build_clean_polygon_3(hole), t.radius() - 0.01);

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

    SECTION("Some cuts need more than 0.01 hp even after shifting the flat region") {
      flat_region shifted_up = r.shift(point(0, 0, 3.2));

      std::vector<toolpath> toolpaths =
	machine_flat_region(shifted_up, 1.0, {t, huge_tool});

      double emco_spindle_hp = 0.737;
      double aluminum_unit_hp = 0.3;

      class region sim_region =
	bounding_region(shifted_up);
      
      bool some_cuts_above_limit = false;
      double limit = 0.01;

      cout << "STARTING HORSEPOWER BOUND TEST" << endl;
      for (auto& tp : toolpaths) {
	cylindrical_bit current_tool(tp.t.cut_diameter());

	for (auto cuts : tp.cuts_without_safe_moves()) {

	  for (auto c : cuts) {
	    double cut_volume = update_cut(*c, sim_region, current_tool);
	    double cut_mrr = cut_volume / cut_execution_time_minutes(c);
	    double cut_power = aluminum_unit_hp*cut_mrr;

	    cout << "Cut draw = " << cut_power << " hp" << endl;

	    if (cut_power > limit) {
	      cout << "Setting cut above limit" << endl;
	      some_cuts_above_limit = true;
	    } else {
	      cout << cut_power << " <= " << limit << endl;
	    }

	  }

	}

      }

      REQUIRE(some_cuts_above_limit);
    }

    SECTION("No cuts exceed the tools power limit after after feed simulation") {
      std::vector<toolpath> toolpaths = machine_flat_region(r, 1.0, {t, huge_tool});

      double emco_spindle_hp = 0.737;
      double aluminum_unit_hp = 0.3;

      optimize_feedrates_by_MRR_simulation(r,
					   toolpaths,
					   emco_spindle_hp,
					   aluminum_unit_hp);

      class region sim_region =
	bounding_region(r);
      
      bool all_cuts_below_power = true;
      for (auto& tp : toolpaths) {
	cylindrical_bit current_tool(tp.t.cut_diameter());

	for (auto cuts : tp.cuts_without_safe_moves()) {

	  for (auto c : cuts) {
	    double cut_volume = update_cut(*c, sim_region, current_tool);
	    double cut_mrr = cut_volume / cut_execution_time_minutes(c);
	    double cut_power = aluminum_unit_hp*cut_mrr;

	    cout << "Cut required power = " << cut_power << " hp" << endl;

	    if (cut_power >= emco_spindle_hp) {
	      all_cuts_below_power = false;
	    }

	  }

	}

      }

      auto prog = build_gcode_program("Surface cut", toolpaths, emco_f1_code_no_TLC);
      cout << prog.name << endl;
      cout << prog.blocks << endl;

      REQUIRE(all_cuts_below_power);

    }
  }

}

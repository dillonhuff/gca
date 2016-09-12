#include "synthesis/mesh_to_gcode.h"
#include "geometry/extrusion.h"

#include <boost/optional.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/vtk_debug.h"
#include "geometry/triangular_mesh.h"
#include "process_planning/feature_selection.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/contour_planning.h"
#include "synthesis/face_clipping.h"
#include "synthesis/jaw_cutout.h"
#include "synthesis/millability.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

  box workpiece_box(const workpiece& w) {
    double x_len = w.sides[0].len();
    double y_len = w.sides[1].len();
    double z_len = w.sides[2].len();
    return box(0, x_len, 0, y_len, 0, z_len);
  }

  // TODO: Dont just make a box
  triangular_mesh stock_mesh(const workpiece& w) {
    box b = workpiece_box(w);
    auto tris = box_triangles(b);
    auto m = make_mesh(tris, 0.001);
    DBG_ASSERT(m.is_connected());
    return m;
  }

  std::vector<plate_height>
  find_viable_parallel_plates(const double aligned_z_height,
			      const double clipped_z_height,
			      const fixtures& f) {
    const vice& v = f.get_vice();
    DBG_ASSERT(!v.has_parallel_plate());

    vector<plate_height> plates;
    for (auto p : f.parallel_plates()) {
      double adjusted_jaw_height = v.jaw_height() - p;
      DBG_ASSERT(adjusted_jaw_height > 0);
      double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
      // TODO: Compute this magic number via friction analysis?
      if (leftover > 0.01 && (clipped_z_height - 0.01) > adjusted_jaw_height) {
	plates.push_back(p);
      }
    }
    return plates;
  }

  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part,
		   const triangular_mesh& out) {
    double work_height = max_in_dir(stock, point(0, 0, 1));
    double part_height = max_in_dir(part, point(0, 0, 1));

    DBG_ASSERT(work_height > part_height);

    auto bound = contour_outline(out.face_indexes(), out, point(0, 0, 1));
    if (bound) {
    } else {
      DBG_ASSERT(false);
    }
    vector<oriented_polygon> outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());

    DBG_ASSERT(outlines.size() == 2);

    oriented_polygon outl =
      min_e(outlines, [](const oriented_polygon& p)
	    { return min_z(p); });

    return face_pocket(work_height, part_height, outl);
  }
  
  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part) {
    double work_height = max_in_dir(stock, point(0, 0, 1));
    double part_height = max_in_dir(part, point(0, 0, 1));

    if (!(work_height > part_height)) {
      cout << "Work height = " << work_height << endl;
      cout << "Part height = " << part_height << endl;
      DBG_ASSERT(work_height > part_height);
    }

    auto bound = contour_outline(stock.face_indexes(), stock, point(0, 0, 1));
    if (bound) {
    } else {
      DBG_ASSERT(false);
    }
    auto outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());
    oriented_polygon outl =
      min_e(outlines, [](const oriented_polygon& p)
	    { return min_z(p); });

    DBG_ASSERT(outlines.size() == 2);

    return face_pocket(work_height, part_height, outl);
  }


  typedef boost::geometry::model::d2::point_xy<double> boost_point_2;
  typedef boost::geometry::model::polygon<boost_point_2> boost_poly_2;
  typedef boost::geometry::model::multi_polygon<boost_poly_2> boost_multipoly_2;
  typedef boost::geometry::model::multi_point<boost_point_2> boost_multipoint_2;

  std::vector<point>
  convex_hull_2D(const std::vector<point>& pts,
		 const double z_level) {
    boost_multipoint_2 mp;
    for (auto p : pts) {
      boost::geometry::append(mp, boost::geometry::model::d2::point_xy<double>(p.x, p.y));
    }

    boost_multipoint_2 res;
    boost::geometry::convex_hull(mp, res);

    vector<point> res_pts;
    for (auto p : res) {
      point pz(p.x(), p.y(), z_level);
      res_pts.push_back(pz);
    }

    return clean_vertices(res_pts);
  }

  
  pocket contour_around(const triangular_mesh& stock,
			const triangular_mesh& part) {
    double stock_top = max_in_dir(stock, point(0, 0, 1));
    double part_top = max_in_dir(part, point(0, 0, 1));

    DBG_ASSERT(stock_top > part_top);

    auto stock_bound =
      contour_outline(stock.face_indexes(), stock, point(0, 0, 1));
    if (stock_bound) {
    } else {
      DBG_ASSERT(false);
    }
    auto stock_outlines =
      mesh_bounds((*stock_bound).index_list(), (*stock_bound).get_parent_mesh());
    DBG_ASSERT(stock_outlines.size() == 2);
    oriented_polygon stock_outline = stock_outlines.front();

    vector<point> part_hull = convex_hull_2D(part.vertex_list(), part_top);
    oriented_polygon part_outline(point(0, 0, 1), part_hull);

    double part_bottom = min_in_dir(part, point(0, 0, 1));    
    return contour_pocket(part_top, part_bottom, part_outline, stock_outline);
  }

  fixture_setup
  clip_top_and_sides(const triangular_mesh& aligned,
		     const triangular_mesh& part,
		     const fixture& f) {
    vector<pocket> pockets{face_down(aligned, part), contour_around(aligned, part)};

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  fixture_setup
  clip_top_and_sides_transform(const triangular_mesh& wp_mesh,
			       const triangular_mesh& part_mesh,
			       feature_decomposition* decomp,
			       const fixture& f,
			       const tool_access_info& tool_info) {
    auto s_t = mating_transform(wp_mesh, f.orient, f.v);

    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    fixture_setup setup = clip_top_and_sides(aligned, part, f);

    std::vector<pocket>& setup_pockets = setup.pockets;

    auto pockets = feature_pockets(*decomp, s_t, tool_info);

    concat(setup_pockets, pockets);
    
    return setup;
  }

  fixture_setup
  clip_base_transform(const triangular_mesh& wp_mesh,
		      const triangular_mesh& part_mesh,
		      feature_decomposition* decomp,
		      const fixture& f,
		      const tool_access_info& tool_info) {
    auto s_t = mating_transform(part_mesh, f.orient, f.v);

    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    fixture_setup setup = clip_base(aligned, part, f);

    std::vector<pocket>& setup_pockets = setup.pockets;

    DBG_ASSERT(setup_pockets.size() == 1);

    auto pockets = feature_pockets(*decomp, s_t, tool_info);

    concat(setup_pockets, pockets);

    return setup;
  }

  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f) {
    vector<pocket> pockets{face_down(aligned, part)};

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  boost::optional<fixture>
  find_top_contour_fixture(const triangular_mesh& aligned,
			   const triangular_mesh& part_mesh,
			   const fixtures& f,
			   const point n) {
    double aligned_z_height = diameter(n, aligned);
    double clipped_z_height = diameter(n, part_mesh);
    vector<plate_height> viable_plates =
      find_viable_parallel_plates(aligned_z_height, clipped_z_height, f);

    cout << "# of viable parallel plates = " << viable_plates.size() << endl;

    if (viable_plates.size() > 0) {
      vice parallel(f.get_vice(), viable_plates.front());

      vector<surface> stock_surfs = outer_surfaces(aligned);

      cout << "n = " << n << endl;
      auto stock_top_orient = largest_upward_orientation(stock_surfs, parallel, n);
      return fixture(stock_top_orient, parallel);
    } else {
      return boost::none;
    }
  }

  // TODO: Add code to generate jaws for base fixture
  boost::optional<fixture>
  find_base_contour_fixture(const surface& outline_of_contour,
			    const surface& top_of_contour,
			    const vice& v,
			    const point n) {

    const triangular_mesh& part_mesh = outline_of_contour.get_parent_mesh();
    auto cont_region_inds =
      const_orientation_regions(part_mesh);

    vector<surface> const_orient_surfs =
      inds_to_surfaces(cont_region_inds, outline_of_contour.get_parent_mesh());

    double total_area = merge_surfaces(const_orient_surfs).surface_area();

    cout << "START all_stable_orientations" << endl;
    std::vector<clamp_orientation> orients =
      all_stable_orientations_with_top_normal(const_orient_surfs, v, n);
    cout << "DONE all_stable_orientations" << endl;

    if (orients.size() > 0) {
      cout << "NO CUSTOM JAW CUTOUT" << endl;
      auto cl = max_e(orients, [part_mesh](const clamp_orientation& c)
		      { return c.contact_area(part_mesh); });
      return fixture(cl, v);
    }

    cout << "Needs custom jaw cutout fixture" << endl;

    return boost::none;

  }
  
  std::vector<fixture_setup>
  contour_clip_setups(const triangular_mesh& aligned,
		      const triangular_mesh& part_mesh,
		      const fixture& top_fix,
		      const fixture& base_fix,
		      const std::vector<tool>& tools) {
    DBG_ASSERT(angle_eps(top_fix.orient.top_normal(), base_fix.orient.top_normal(), 180.0, 1.0));

    auto features = select_features(aligned, part_mesh, {top_fix, base_fix}, tools);

    DBG_ASSERT(features.decomps.size() == 2);
    DBG_ASSERT(features.access_info.size() == 2);

    auto top_decomp = features.decomps[0];
    auto base_decomp = features.decomps[1];

    // vtk_debug_features(collect_features(top_decomp));
    // vtk_debug_features(collect_features(base_decomp));

    const auto& top_tool_info = features.access_info[0];
    const auto& base_tool_info = features.access_info[1];

    std::vector<fixture_setup> clip_setups;
    clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, top_decomp, top_fix, top_tool_info));
    clip_setups.push_back(clip_base_transform(aligned, part_mesh, base_decomp, base_fix, base_tool_info));

    return clip_setups;
  }

  
  clipping_plan
  base_fix_clip_plan(const workpiece& w,
		     const triangular_mesh& aligned,
		     const triangular_mesh& part_mesh,
		     const contour_surface_decomposition& surfs,
		     const fixture& top_fix,
		     const fixture& base_fix,
		     const std::vector<tool>& tools) {
    std::vector<fixture_setup> clip_setups =
      contour_clip_setups(aligned, part_mesh, top_fix, base_fix, tools);
    
    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, aligned);
    auto surfs_to_cut = surfs.rest;

    return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, w);
  }

  boost::optional<clipping_plan>
  parallel_plate_clipping(const triangular_mesh& part_mesh,
			  const fabrication_inputs& fab_inputs) {

    cout << "Trying parallel plate clipping" << endl;

    const auto& f = fab_inputs.f;

    DBG_ASSERT(fab_inputs.w.size() > 0);

    const auto& w = fab_inputs.w.front();

    vector<surface> stable_surfaces = outer_surfaces(part_mesh);
    triangular_mesh aligned = align_workpiece(stable_surfaces, w);

    boost::optional<contour_surface_decomposition> surfs =
      compute_contour_surfaces(part_mesh);

    if (surfs) {
      point n = surfs->n;
      const surface& outline = surfs->outline;
      const surface& top = surfs->top;
      
      cout << "Has outline and flat top in " << n << endl;

      boost::optional<fixture> top_fix =
      	find_top_contour_fixture(aligned, part_mesh, f, n);

      if (top_fix) {
	cout << "Has top fix in " << n << endl;

	boost::optional<fixture> base_fix =
	  find_base_contour_fixture(outline, top, (*top_fix).v, -1*n);

	if (base_fix) {
	  return base_fix_clip_plan(w, aligned, part_mesh, *surfs, *top_fix, *base_fix, fab_inputs.tools);
	} else {
	  return custom_jaw_plan(w, aligned, part_mesh, *surfs, *top_fix, fab_inputs);
	}
      }
    }

    return boost::none;
  }

  clipping_plan
  workpiece_clipping_programs(const std::vector<workpiece>& wps,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& f) {

    DBG_ASSERT(wps.size() > 0);

    auto contour_clip =
      parallel_plate_clipping(part_mesh, fabrication_inputs{f, tools, wps});

    if (contour_clip) {
      cout << "Contouring" << endl;
      return *contour_clip;
    } else {
      return axis_by_axis_clipping(wps, part_mesh, tools, f);
    }
  }

}

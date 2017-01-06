#include <random>

#include <boost/numeric/ublas/io.hpp>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/rotation.h"
#include "geometry/surface.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "synthesis/contour_planning.h"
#include "utils/arena_allocator.h"
#include "utils/check.h"

namespace gca {

  gca::feature* node_value(feature_decomposition* f) { return f->feature(); }

  polygon_3
  convex_hull_2D(const triangular_mesh& m,
		 const point n) {
    double z_level = max_distance_along(m.vertex_list(), n);

    return convex_hull_2D(m.vertex_list(), n, z_level);
  }

  std::vector<point> projected_hull(const plane pl,
				    const std::vector<point>& raw_pts) {
    auto pts = project_points(pl, raw_pts);

    polygon_3 p =
      convex_hull_2D(pts, pl.normal(), max_distance_along(pts, pl.normal()));

    check_simplicity(p);

    p.correct_winding_order(pl.normal());

    return p.vertices();
  }

  std::vector<std::vector<index_t>>
  not_vertical_or_horizontal_regions(const triangular_mesh& m,
				     const std::vector<std::vector<index_t>>& surfs,
				     const point n) {
    auto not_vert_or_horiz =
      select(surfs, [n, m](const std::vector<index_t>& s) {
	  return !all_parallel_to(s, m, n, 0.05) &&
	  !all_orthogonal_to(s, m, n, 0.05) &&
	  !all_antiparallel_to(s, m, n, 0.05);
	});

    // Cull backfaces
    delete_if(not_vert_or_horiz,
	      [n, m](const std::vector<index_t>& inds) {
		return angle_between(m.face_orientation(inds.front()), n) > 90.0;
	      });

    

    vector<index_t> vz;
    for (auto s : not_vert_or_horiz) {
      concat(vz, s);
    }

    if (vz.size() == 0) { return {}; }

    return connect_regions(vz, m);
  }

  point random_point(std::mt19937& gen, const double tol) {
    std::uniform_real_distribution<> dis(-tol, tol);

    double r_x = dis(gen);
    double r_y = dis(gen);
    double r_z = dis(gen);

    return point(r_x, r_y, r_z);
  }

  polygon_3
  virtual_surface_for_surface(const std::vector<index_t>& s,
			      const triangular_mesh& m,
			      const point n) {
    DBG_ASSERT(s.size() > 0);

    //vtk_debug_highlight_inds(s, m);

    cout << "# of triangles initially = " << s.size() << endl;

    auto raw_pts = vertexes_on_surface(s, m);
    point max_pt = max_along(raw_pts, n);
    plane top(n, max_pt);

    std::vector<polygon_3> ts;
    for (auto i : s) {
      triangle t = m.face_triangle(i);
      vector<point> pts = project_points(top, {t.v1, t.v2, t.v3});
      polygon_3 l(pts, true);
      
      ts.push_back(l);

    }

    cout << "# of triangle polygons = " << ts.size() << endl;

    //vtk_debug_highlight_inds(s, m);

    cout << "Trying planar union for the first time" << endl;
    std::vector<polygon_3> result_polys =
      planar_polygon_union(n, ts);
    cout << "Done with planar union for the first time" << endl;

    if (!(result_polys.size() == 1)) {

      //vtk_debug_highlight_inds(s, m);
      vector<oriented_polygon> bounds = mesh_bounds(s, m);

      if (bounds.size() != 1) {
	vtk_debug_polygons(bounds);
	DBG_ASSERT(bounds.size() == 1);
      }

      auto projected = project_points(top, bounds.front().vertices());
      auto projected_clean = clean_ring_for_offsetting_no_fail(projected);

      DBG_ASSERT(projected_clean.size() > 2);

      polygon_3 bound_p =
	build_clean_polygon_3(projected_clean);
      bound_p.correct_winding_order(n);

      
      return bound_p;
    }

    return result_polys.front();
  }

  std::vector<polygon_3>
  build_virtual_surfaces(const triangular_mesh& m,
			 const std::vector<index_t>& indexes,
			 const point n) {

    auto surfs = const_orientation_regions(m);
    auto not_vertical_or_horizontal =
      not_vertical_or_horizontal_regions(m, surfs, n);

    cout << "# of virtual surfaces = " << not_vertical_or_horizontal.size() << endl;

    vector<polygon_3> polys;
    for (auto s : not_vertical_or_horizontal) {
      auto l = virtual_surface_for_surface(s, m, n);
      polys.push_back(l);
    }

    return polys;
  }

  std::vector<polygon_3>
  horizontal_surfaces(const triangular_mesh& m, const point n) {
    auto inds = m.face_indexes();

    auto horiz_inds =
      select(inds, [m, n](const index_t i)
	     { return angle_eps(n, m.face_orientation(i), 0.0, 0.1); });

    auto virtual_surfaces =
      build_virtual_surfaces(m, inds, n);
    
    //vtk_debug_highlight_inds(horiz_inds, m);    

    // TODO: Does angle matter now? It shouldnt
    auto surfs = normal_delta_regions(horiz_inds, m, 3.0);

    cout << "# of horizontal surfaces in " << n << " = " << surfs.size() << endl;
    // for (auto s : surfs) {
    //   vtk_debug_highlight_inds(s, m);
    // }

    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    triangular_mesh mr = apply(r, m);

    vector<polygon_3> surf_polys = virtual_surfaces;
    for (auto s : surfs) {
      DBG_ASSERT(s.size() > 0);

      auto bounds = surface_boundary_polygons(s, mr);

      DBG_ASSERT(bounds.size() == 1);

      polygon_3 lp = apply(r_inv, bounds.front());
      lp.correct_winding_order(n);

      surf_polys.push_back(lp);
    }

    cout << "Got horizontal surfaces" << endl;

    vector<polygon_3> cleaned_surf_polys;
    for (auto& p : surf_polys) {
      boost::optional<polygon_3> cleaned =
	clean_polygon_for_offsetting_maybe(p);
      if (cleaned) {
	cleaned_surf_polys.push_back(*cleaned);
      }
    }

    return cleaned_surf_polys;
  }

  surface_levels
  build_surface_levels(const std::vector<polygon_3>& h_surfs,
		       const point n) {
    auto surface_it = begin(h_surfs);
    surface_levels levels;
    while (surface_it != end(h_surfs)) {
      const polygon_3& current_surf = *surface_it;
      double current_surf_dist = max_distance_along(current_surf.vertices(), n);
      auto next_it =
	find_if_not(surface_it, end(h_surfs),
		    [current_surf_dist, n](const labeled_polygon_3& p) {
		      double pd = max_distance_along(p.vertices(), n);
		      return within_eps(current_surf_dist, pd, 0.01);
		    });
      vector<labeled_polygon_3> level(surface_it, next_it);
      levels.push_back(level);

      surface_it = next_it;
    }

    return levels;
  }

  surface_levels
  initial_surface_levels(const std::vector<triangular_mesh>& meshes,
			 const point n) {
    vector<polygon_3> surf_polys;
    for (auto& m : meshes) {
      concat(surf_polys, horizontal_surfaces(m, n));
    }

    sort(begin(surf_polys), end(surf_polys),
	 [n](const polygon_3& x, const polygon_3& y) {
	   return max_distance_along(x.vertices(), n) <
	     max_distance_along(y.vertices(), n);
	 });

    return build_surface_levels(surf_polys, n);
  }

  labeled_polygon_3 initial_outline(const triangular_mesh& m,
				    const point n) {

    labeled_polygon_3 top_and_bottom_outline =
      convex_hull_2D(m, n);

    point top_point = max_point_in_dir(m, n);
    plane max_plane(n, top_point);

    labeled_polygon_3 top_poly = project_onto(max_plane, top_and_bottom_outline);

    check_simplicity(top_poly.vertices());
    
    top_poly.correct_winding_order(n);

    // TODO: Shift plane eps
    plane eps_pl(top_poly.normal(), top_poly.vertex(0) + 0.00001*top_poly.normal());
    return project_onto(eps_pl, top_poly);
  }

  boost::optional<std::vector<labeled_polygon_3>>
  subtract_level(const labeled_polygon_3& p,
		 const std::vector<labeled_polygon_3>& to_subtract) {
    DBG_ASSERT(to_subtract.size() > 0);

    cout << "Subtracting" << endl;

    //vtk_debug_polygon(p);
    //vtk_debug_polygons(to_subtract);

    double level_z =
      max_distance_along(to_subtract.front().vertices(), p.normal());
    point n = to_subtract.front().normal();

    cout << "n = " << n << endl;
    
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    // cout << "rotation = " << r << endl;
    // cout << "r*n = " << times_3(r, n) << endl;

    boost_poly_2 pb = to_boost_poly_2(apply(r, p));

    auto to_subtract_dilated = dilate_polygons(to_subtract, 0.00001);

    cout << "# polys to subtract = " << to_subtract_dilated.size() << endl;

    boost_multipoly_2 result;
    result.push_back(pb);

    for (auto s : to_subtract_dilated) {
      auto bp = to_boost_poly_2(apply(r, s));
      boost_multipoly_2 r_tmp = result;
      boost::geometry::clear(result);
      boost::geometry::difference(r_tmp, bp, result);
    }

    cout << "# polys in result = " << result.size() << endl;

    bool result_same = false;

    if (result.size() > 0) {
      auto lr =
    	max_element(begin(result), end(result),
    		    [](const boost_poly_2& l, const boost_poly_2& r)
    		    { return boost::geometry::area(l) < boost::geometry::area(r); });

      auto& largest_result = *lr;

      // The result is the same as before
      if (within_eps(boost::geometry::area(largest_result),
    		     boost::geometry::area(pb),
    		     0.0005)) {
	cout << "Result is the same" << endl;
	result_same = true;
    	//return boost::none;
      }
    }

    // TODO: How to preserve edge labels etc?
    std::vector<labeled_polygon_3> res;
    for (auto r : result) {
      if (boost::geometry::area(r) > 0.001) {
	labeled_polygon_3 lp = to_labeled_polygon_3(r_inv, level_z, r);

	check_simplicity(lp);

	lp.correct_winding_order(p.normal());
	res.push_back(lp);
      }
    }

    //cout << "RESULT OF SUBTRACTION" << endl;
    //vtk_debug_polygons(res);

    if (result_same) { return boost::none; }


    // TODO: Add feature patching here?
    return res;
  }

  void
  decompose_volume(const labeled_polygon_3& current_level,
		   surface_levels levels,
		   feature_decomposition* parent,
		   const double base_depth) {
    cout << "decompose" << endl;
    cout << "Levels left = " << levels.size() << endl;

    double current_depth =
      max_distance_along(current_level.vertices(), current_level.normal());

    cout << "current normal = " << current_level.normal() << endl;
    cout << "current depth  = " << current_depth << endl;
    cout << "base depth     = " << base_depth << endl;

    DBG_ASSERT(current_depth >= base_depth);
    
    //    vtk_debug_polygon(current_level);
    
    // If there are no levels left then the current_level defines
    // a through feature
    if (levels.size() == 0) {
      double feature_depth = current_depth - base_depth;

      DBG_ASSERT(feature_depth > 0.0);

      point v = feature_depth*current_level.normal();
      plane base_pl(current_level.normal(), current_level.vertex(0) - v);

      labeled_polygon_3 shifted = project_onto(base_pl, current_level);

      feature* f = new (allocate<feature>()) feature(true, true, feature_depth, shifted);
      feature_decomposition* child =
    	new (allocate<feature_decomposition>()) feature_decomposition(f);
      parent->add_child(child);

      return;
    }

    const std::vector<labeled_polygon_3>& level_polys = levels.back();

    boost::optional<vector<labeled_polygon_3>> r_polys =
      subtract_level(current_level, level_polys);

    double next_depth =
      max_distance_along(levels.back().front().vertices(),
			 levels.back().front().normal());

    levels.pop_back();

    // The result is exactly the same as before, just recurse with no
    // updates
    if (!r_polys) {
      decompose_volume(current_level, levels, parent, base_depth);
      return;
    }

    auto result_polys = *r_polys;

    //cout << "Subtraction result" << endl;
    //vtk_debug_polygons(result_polys);

    // Add a new feature for the current level
    // and recursively build decompositions for each new level
    // produced by the subtraction
    cout << "Current depth = " << current_depth << endl;
    cout << "Next depth    = " << next_depth << endl;

    if (!(current_depth >= next_depth)) {
      cout << "Direction = " << current_level.normal() << endl;
      cout << "current_depth = " << current_depth << endl;
      cout << "next_depth    = " << next_depth << endl;
      
      vtk_debug_polygon(current_level);
      for (auto p : *r_polys) {
	vtk_debug_polygon(p);
      }

      DBG_ASSERT(false);
    }

    double feature_depth = current_depth - next_depth;

    DBG_ASSERT(feature_depth > 0.0);

    point v = feature_depth*current_level.normal();
    plane base_pl(current_level.normal(), current_level.vertex(0) - v);

    labeled_polygon_3 shifted = project_onto(base_pl, current_level);
    
    feature* f = new (allocate<feature>()) feature(true, false, feature_depth, shifted);
    feature_decomposition* child =
      new (allocate<feature_decomposition>()) feature_decomposition(f);
    parent->add_child(child);

    //vector<feature_decomposition*> children;
    for (auto r : result_polys) {
      decompose_volume(r, levels, child, base_depth);
    }

  }

  void check_normals(feature_decomposition* decomp, const point n) {
    auto check_normal = [n](const feature* f) {
      if (f != nullptr) {
	DBG_ASSERT(f->depth() >= 0);
	DBG_ASSERT(within_eps(angle_between(n, f->normal()), 0.0, 1.0));
      }
    };

    traverse_bf(decomp, check_normal);
  }

  void
  set_open_features(feature_decomposition* decomp) {
    DBG_ASSERT(decomp->num_children() == 1);

    feature_decomposition* top = decomp->child(0);
    vector<point> stock_ring = top->feature()->base().vertices();
    polygon_3 stock_polygon =
      build_clean_polygon_3(stock_ring);

    auto set_open = [stock_polygon](feature* f) {
      if (f != nullptr) {
	if (is_outer(*f, stock_polygon)) {
	  f->set_closed(false);
	}
      }
    };

    traverse_bf(decomp, set_open);
  }

  void 
  check_level_sizes(const surface_levels& levels) {

    for (auto level : levels) {
      
      DBG_ASSERT(level.size() > 0);

      cout << "LEVEL" << endl;
      for (auto l : level) {
	double current_depth =
	  max_distance_along(l.vertices(),
			     l.normal());
	cout << "----- Current depth = " << current_depth << endl;
      }

    }

  }

  void 
  check_level_depths(const polygon_3& init_outline,
		     const surface_levels& levels) {
    if (levels.size() > 0) {    
      for (unsigned i = 0; i < levels.size() - 1; i++) {
	auto& current = levels[i + 1];
	auto& next = levels[i];

	double current_depth =
	  max_distance_along(current.front().vertices(),
			     current.front().normal());

	double next_depth =
	  max_distance_along(next.front().vertices(),
			     next.front().normal());

	cout << "Current depth = " << current_depth << endl;
	cout << "Next depth    = " << next_depth << endl;

	DBG_ASSERT(current_depth >= next_depth);
      }

      double current_depth =
	max_distance_along(init_outline.vertices(), init_outline.normal());

      cout << "current depth  = " << current_depth << endl;

      double next_depth =
	max_distance_along(levels.back().front().vertices(),
			   levels.back().front().normal());

      cout << "Current depth = " << current_depth << endl;
      cout << "Next depth    = " << next_depth << endl;

      DBG_ASSERT(current_depth >= next_depth);
    }
    
  }

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& stock,
			      const std::vector<triangular_mesh>& meshes,
			      const point n) {
    labeled_polygon_3 init_outline = initial_outline(stock, n);

    cout << "STARTING FEATURE DECOMPOSITION IN " << n << endl;

    DBG_ASSERT(within_eps(angle_between(init_outline.normal(), n), 0.0, 0.01));

    triangular_mesh lowest = min_e(meshes, [n](const triangular_mesh& m) {
	return min_distance_along(m.vertex_list(), n);
      });
    double base_depth = min_distance_along(lowest.vertex_list(), n);

    surface_levels levels = initial_surface_levels(meshes, n);

    check_level_sizes(levels);
    check_level_depths(init_outline, levels);

    feature_decomposition* decomp =
      new (allocate<feature_decomposition>()) feature_decomposition();
    decompose_volume(init_outline, levels, decomp, base_depth);

    check_normals(decomp, n);

    set_open_features(decomp);

    return decomp;
  }
  

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& stock,
			      const triangular_mesh& m,
			      const point n) {
    vector<triangular_mesh> meshes{m};
    return build_feature_decomposition(stock, meshes, n);
  }

  // Uses the part itself as an outline
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m,
			      const point n) {
    return build_feature_decomposition(m, m, n);
  }


  vector<feature*> collect_features(feature_decomposition* f) {
    DBG_ASSERT(f != nullptr);
    
    vector<feature*> features;

    auto func = [&features](feature* f) {
      if (f != nullptr)
	{ features.push_back(f); }
    };

    traverse_bf(f, func);

    return features;
  }

  vector<feature*> collect_leaf_features(feature_decomposition* f) {
    DBG_ASSERT(f != nullptr);
    
    vector<feature*> features;

    auto func = [&features](feature* f) {
      if (f != nullptr)
	{ features.push_back(f); }
    };

    traverse_leaves_bf(f, func);

    return features;
  }
  
  point normal(feature_decomposition* f) {
    point n;
    auto check_normal = [&n](const feature* f) {
      if (f != nullptr) {
	n = f->normal();
      }
    };

    traverse_bf(f, check_normal);

    return n;
  }

  feature* parent_feature(feature* f, feature_decomposition* decomp) {
    for (unsigned i = 0; i < decomp->num_children(); i++) {
      feature* child_feature = decomp->child(i)->feature();

      if (f == child_feature) { return decomp->feature(); }
    }

    for (unsigned i = 0; i < decomp->num_children(); i++) {
      feature* possible_parent = parent_feature(f, decomp->child(i));

      if (possible_parent != nullptr) { return possible_parent; }
    }

    return nullptr;
  }

  // TODO: Version of this code that can handle holes?
  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p) {
    return oriented_polygon(p.normal(), p.vertices());
  }

}

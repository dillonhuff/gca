#include "feature_recognition/prismatic_feature_utils.h"
#include "geometry/extrusion.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"

namespace gca {

  // NOTE: Perhaps all we need to do is find a viable hull for the remaining feature?
  boost::optional<feature>
  extract_feature(const feature& original,
		  const triangular_mesh& portion) {

    point n = original.normal();

    cout << "NORMAL = " << n << endl;

    //vtk_debug_mesh(portion);

    vector<index_t> inds = portion.face_indexes();

    auto top = select(inds, [portion, n](const index_t i) {
    	return angle_eps(portion.face_orientation(i), n, 0.0, 1.0);
      });

    auto top_cpy = top;
    auto top_regions = normal_delta_regions(top_cpy, portion, 180.0);

    // cout << "TOP" << endl;
    // cout << portion.face_orientation(top.front()) << endl;
    // vtk_debug_highlight_inds(top, portion);

    if (top_regions.size() != 1) {
      cout << "# of top regions = " << top_regions.size() << endl;
      return boost::none;
    }

    subtract(inds, top);

    auto bottom = select(inds, [portion, n](const index_t i) {
	return angle_eps(portion.face_orientation(i), n, 180.0, 1.0);
      });

    auto bottom_cpy = bottom;
    auto bottom_regions = normal_delta_regions(bottom_cpy, portion, 180.0);

    // cout << "BOTTOM" << endl;
    // cout << portion.face_orientation(bottom.front()) << endl;
    // vtk_debug_highlight_inds(bottom, portion);

    if (bottom_regions.size() != 1) {
      cout << "# of bottom regions = " << bottom_regions.size() << endl;
      return boost::none;
    }

    subtract(inds, bottom);

    // cout << "REST" << endl;
    // vtk_debug_highlight_inds(inds, portion);

    bool all_other_triangles_vertical =
      all_of(begin(inds), end(inds), [portion, n](const index_t i) {
	  return angle_eps(portion.face_orientation(i), n, 90.0, 1.0);
	});

    if (!all_other_triangles_vertical) {
      cout << "All other triangles not vertical" << endl;
      return boost::none;
    }

    // vector<oriented_polygon> bounds = mesh_bounds(bottom, portion);
    // vector<vector<point>> rings;
    // for (auto& b : bounds) {
    //   rings.push_back(b.vertices());
    // }

    vector<polygon_3> polys = surface_boundary_polygons(bottom, portion);
    //arrange_rings(rings);


    if (polys.size() != 1) {
      // vtk_debug_polygons(polys);
      // DBG_ASSERT(polys.size() == 1);

      DBG_ASSERT(polys.size() > 0);
      return boost::none;
    }

    cout << "Using split feature" << endl;
    if (polys.front().holes().size() == original.base().holes().size()) {
      feature clipped_feature(original.is_closed(), original.is_through(), original.depth(), polys.front());
      return clipped_feature;
    } else {
      feature clipped_feature(true, false, original.depth(), polys.front());
      return clipped_feature;
    }

  }

  feature
  extract_extrusion_feature(const point n,
			    const triangular_mesh& m) {

    cout << "Extracting extrusion feature" << endl;
    cout << "NORMAL = " << n << endl;

    //vtk_debug_mesh(m);

    vector<index_t> inds = m.face_indexes();

    auto top = select(inds, [m, n](const index_t i) {
    	return angle_eps(m.face_orientation(i), n, 0.0, 1.0);
      });

    auto top_cpy = top;
    auto top_regions = normal_delta_regions(top_cpy, m, 180.0);

    cout << "TOP" << endl;
    cout << m.face_orientation(top.front()) << endl;
    //vtk_debug_highlight_inds(top, m);

    if (top_regions.size() != 1) {
      cout << "# of top regions = " << top_regions.size() << endl;
      DBG_ASSERT(false);
    }

    subtract(inds, top);

    auto bottom = select(inds, [m, n](const index_t i) {
	return angle_eps(m.face_orientation(i), n, 180.0, 1.0);
      });

    auto bottom_cpy = bottom;
    auto bottom_regions = normal_delta_regions(bottom_cpy, m, 180.0);

    // cout << "BOTTOM" << endl;
    // cout << portion.face_orientation(bottom.front()) << endl;
    // vtk_debug_highlight_inds(bottom, portion);

    if (bottom_regions.size() != 1) {
      cout << "# of bottom regions = " << bottom_regions.size() << endl;
      DBG_ASSERT(false);
    }

    subtract(inds, bottom);

    // cout << "REST" << endl;
    // vtk_debug_highlight_inds(inds, portion);

    bool all_other_triangles_vertical =
      all_of(begin(inds), end(inds), [m, n](const index_t i) {
	  return angle_eps(m.face_orientation(i), n, 90.0, 1.0);
	});

    if (!all_other_triangles_vertical) {
      cout << "All other triangles not vertical" << endl;
      DBG_ASSERT(false);
    }

    vector<polygon_3> polys = surface_boundary_polygons(bottom, m);

    if (polys.size() != 1) {
      vtk_debug_polygons(polys);

      DBG_ASSERT(false);
    }

    // TODO: Fix this hack, it may not actually work in all cases
    polygon_3 base_poly = build_clean_polygon_3(polys.front().vertices());
    double depth = max_in_dir(m, n) - min_in_dir(m, n);
    feature result(true, false, depth, base_poly);

    //vtk_debug_feature(result);

    return result;
  }

  double volume(const feature& f) {
    const rotation r = rotate_from_to(f.normal(), point(0, 0, 1));
    auto bp = to_boost_poly_2(apply(r, f.base()));

    double base_area = bg::area(bp);
    
    return base_area * f.depth();
  }

  triangular_mesh feature_mesh(const feature& f,
			       const double base_dilation,
			       const double top_extension,
			       const double base_extension) {
    point shift_vec = (-1*base_extension)*f.normal();

    if (base_dilation > 0.0) {
      auto dilated_base = dilate(f.base(), base_dilation);
      dilated_base = shift(shift_vec, dilated_base);
    
      auto m = extrude(dilated_base, (top_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;

    } else {
      auto base = shift(shift_vec, f.base());

      auto m = extrude(base, (top_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;
    }
  }

  triangular_mesh feature_mesh(const feature& f) {

    return feature_mesh(f, 0.0, 0.0001, 0.0000); 
  }

  int curve_count(const feature& f) {
    return curve_count(f.base());
  }

}

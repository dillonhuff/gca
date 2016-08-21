#include "feature_recognition/feature_decomposition.h"
#include "geometry/surface.h"
#include "synthesis/contour_planning.h"
#include "utils/arena_allocator.h"
#include "utils/check.h"

namespace gca {

  labeled_polygon_3 slide_onto(const plane p,
			       const labeled_polygon_3& poly) {
    point n = poly.normal();

    if (!within_eps(n, p.normal(), 0.01)) {
      n = -1*n;
    }

    cout << "poly normal  = " << n << endl;
    cout << "plane normal = " << p.normal() << endl;

    DBG_ASSERT(within_eps(n, p.normal(), 0.01));

    point v = project_onto(p.pt() - poly.vertex(0), n);

    vector<point> verts;
    for (unsigned i = 0; i < poly.num_vertices(); i++) {
      verts.push_back(poly.vertex(i) + v);
    }

    return labeled_polygon_3(verts);
  }

  std::vector<labeled_polygon_3>
  horizontal_surfaces(const triangular_mesh& m, const point n) {
    vector<std::vector<index_t>> surfs = const_orientation_regions(m);

    // TODO: Add virtual polygons for surfaces that are non horizontal and
    // non vertical
    filter_non_horizontal_surfaces_wrt_dir(surfs, m, n);

    cout << "# of horizontal surfaces in " << n << " = " << surfs.size() << endl;

    vector<labeled_polygon_3> surf_polys;
    for (auto s : surfs) {
      DBG_ASSERT(s.size() > 0);

      auto bounds = mesh_bounds(s, m);

      DBG_ASSERT(bounds.size() > 0);

      auto boundary = extract_boundary(bounds);

      DBG_ASSERT(area(boundary) > 0.001);

      auto holes = bounds;

      vector<vector<point>> hole_verts;
      for (auto h : holes) {
	hole_verts.push_back(h.vertices());
      }

      surf_polys.push_back(labeled_polygon_3(boundary.vertices(), hole_verts));
    }
    
    return surf_polys;
  }

  surface_levels
  initial_surface_levels(const triangular_mesh& m,
			 const point n) {
    auto h_surfs = horizontal_surfaces(m, n);
    DBG_ASSERT(false);
  }

  labeled_polygon_3 initial_outline(const triangular_mesh& m,
				    const point n) {
    boost::optional<oriented_polygon> out = simple_outline(m, n);

    DBG_ASSERT(out);

    labeled_polygon_3 top_and_bottom_outline(out->vertices());

    point top_point = max_point_in_dir(m, n);
    plane max_plane(n, top_point);

    labeled_polygon_3 top_poly = slide_onto(max_plane, top_and_bottom_outline);

    return top_poly;
  }

  feature_decomposition*
  decompose_volume(const labeled_polygon_3& current_level,
		   surface_levels& levels,
		   feature_decomposition* parent) {
    if (levels.size() == 0) { return parent; }

    const std::vector<labeled_polygon_3> level_polys = levels.back();
    vector<labeled_polygon_3> result_polys = ;
    return parent;
  }
  
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n) {
    labeled_polygon_3 init_outline = initial_outline(m, n);
    surface_levels levels = initial_surface_levels(m, n);

    feature_decomposition* empty_decomp =
      new (allocate<feature_decomposition>()) feature_decomposition();

    return decompose_volume(init_outline, levels, empty_decomp);
  }

}

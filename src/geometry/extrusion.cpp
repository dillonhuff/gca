#include "geometry/extrusion.h"

namespace gca {

  oriented_polygon
  convert_index_poly(const std::vector<point>& pts,
		     const index_poly& p,
		     const double d,
		     const point e) {
    vector<point> poly_pts;
    for (auto i : p) {
      poly_pts.push_back(pts[i] + d*e.normalize());
    }
    return oriented_polygon(e, poly_pts);
  }

  triangular_mesh
  mesh_for_polys(const std::vector<oriented_polygon>& polys) {
    std::vector<triangle> tris;
    for (auto p : polys) {
      concat(tris, vtk_triangulate_poly(p));
    }
    return make_mesh(tris, 0.01);
  }

  oriented_polygon
  side_rect(const std::vector<point>& pts,
	    const gca::edge e,
	    const double d0,
	    const double d1,
	    const point dir) {
    assert(d1 > d0);
    point i0 = d0*dir.normalize();
    point i1 = d1*dir.normalize();
    point top_left = pts[e.l] + i0;
    point top_right = pts[e.r] + i0;
    point bottom_left = pts[e.l] + i1;
    point bottom_right = pts[e.r] + i1;
    std::vector<point> poly_points{top_left, top_right, bottom_right, bottom_left};
    // TODO: Proper normal computation
    return oriented_polygon(dir, poly_points);
  }
  
  std::vector<oriented_polygon>
  side_polys(const std::vector<point>& pts,
	     const index_poly& p,
	     const double d0,
	     const double d1,
	     const point e) {
    std::vector<gca::edge> edges;
    for (index_t i = 0; i < p.size(); i++) {
      index_t j = (i + 1) % p.size();
      edges.push_back(edge(i, j));
    }
    std::vector<oriented_polygon> side_rects;
    for (auto ed : edges) {
      side_rects.push_back(side_rect(pts, ed, d0, d1, e));
    }
    return side_rects;
  }

  triangular_mesh
  extrude_layers(const std::vector<point>& pts,
		 const std::vector<index_poly>& poly_layers,
		 const std::vector<double>& layer_depths,
		 const point extrude_dir) {
    assert(poly_layers.size() == 1);
    std::vector<oriented_polygon> polys;
    polys.push_back(convert_index_poly(pts, poly_layers[0], 0, extrude_dir));
    polys.push_back(convert_index_poly(pts, poly_layers[0], layer_depths[0], extrude_dir));
    concat(polys, side_polys(pts, poly_layers[0], 0, layer_depths[0], extrude_dir));
    return mesh_for_polys(polys);
  }

}

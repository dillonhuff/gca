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

  triangular_mesh
  extrude_layers(const std::vector<point>& pts,
		 const std::vector<index_poly>& poly_layers,
		 const std::vector<double>& layer_depths,
		 const point extrude_dir) {
    assert(poly_layers.size() == 1);
    std::vector<oriented_polygon> polys;
    polys.push_back(convert_index_poly(pts, poly_layers[0], 0, extrude_dir));
    polys.push_back(convert_index_poly(pts, poly_layers[0], layer_depths[0], -1*extrude_dir));
    return mesh_for_polys(polys);
  }

}

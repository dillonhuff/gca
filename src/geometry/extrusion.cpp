#include "geometry/vtk_debug.h"
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

    vtk_debug_triangles(tris);
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
    std::vector<point> poly_points{bottom_left, bottom_right, top_right, top_left};
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
      edges.push_back(edge(p[i], p[j]));
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
    assert(pts.size() > 0);
    assert(layer_depths.size() == poly_layers.size());
    std::vector<oriented_polygon> polys;

    double last_depth_offset = 0.0;
    for (unsigned i = 0; i < layer_depths.size(); i++) {
      // Front polys
      if (i == 0) {
	polys.push_back(convert_index_poly(pts, poly_layers[i], last_depth_offset, extrude_dir));
      }

      // Sides
      concat(polys, side_polys(pts, poly_layers[i], last_depth_offset, layer_depths[i], extrude_dir));

      // Back
      if (i + 1 == layer_depths.size()) {
	oriented_polygon back =
	  convert_index_poly(pts, poly_layers[i], layer_depths[i], extrude_dir);
	vector<point> verts = back.vertices();
	reverse(begin(verts), end(verts));
	oriented_polygon back_rev(-1*extrude_dir, verts);
	polys.push_back(back_rev);
      }

      last_depth_offset += layer_depths[i];
    }
    return mesh_for_polys(polys);
  }

}

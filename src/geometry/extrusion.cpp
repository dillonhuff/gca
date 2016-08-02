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
    cout << "# of polygons = " << polys.size() << endl;
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

  std::vector<gca::edge>
  index_poly_edges(const index_poly& p) {
    std::vector<gca::edge> edges;
    for (index_t i = 0; i < p.size(); i++) {
      index_t j = (i + 1) % p.size();
      edges.push_back(edge(p[i], p[j]));
    }
    return edges;
  }
  
  std::vector<oriented_polygon>
  side_polys(const std::vector<point>& pts,
	     const index_poly& p,
	     const double d0,
	     const double d1,
	     const point e) {
    auto edges = index_poly_edges(p);
    std::vector<oriented_polygon> side_rects;
    for (auto ed : edges) {
      side_rects.push_back(side_rect(pts, ed, d0, d1, e));
    }
    return side_rects;
  }

  template<typename T>
  std::vector<T>
  symmetric_difference(const std::vector<T>& a,
		       const std::vector<T>& b) {
    auto ac = a;
    subtract(ac, b);

    auto bc = b;
    subtract(bc, a);

    concat(ac, bc);

    return ac;
  }

  // Templatize and merge with collect polygon in triangle.cpp?
  index_poly collect_polygon(vector<gca::edge>& lines) {
    assert(lines.size() > 0);
    vector<index_t> points;
    vector<gca::edge> to_remove;
    points.push_back(lines.front().l);
    points.push_back(lines.front().r);
    lines.erase(lines.begin());
    unsigned i = 0;
    while (lines.size() > 0 && i < lines.size()) {
      if (lines[i].l == points.back()) { //within_eps(lines[i].start, points.back())) {
    	if (lines[i].r == points.front()) {//within_eps(lines[i].end, points.front())) {
    	  lines.erase(lines.begin() + i);	  
    	  return points;
    	}
    	points.push_back(lines[i].r); //lines[i].end);
    	lines.erase(lines.begin() + i);
    	i = 0;
      } else if (lines[i].r == points.back()) {//within_eps(lines[i].end, points.back())) {
    	if (lines[i].l == points.front()) {//within_eps(lines[i].start, points.front())) {
    	  lines.erase(lines.begin() + i);
    	  return points;
    	}
    	points.push_back(lines[i].l); //lines[i].start);
    	lines.erase(lines.begin() + i);
    	i = 0;
      } else {
    	i++;
      }
    }
    return points;
  }

  std::vector<index_poly>
  unordered_segments_to_index_polygons(std::vector<gca::edge>& lines) {
    vector<index_poly> ps;
    auto ls = lines;
    while (ls.size() > 0) {
      index_poly vertices = collect_polygon(ls);
      ps.push_back(vertices);
    }
    return ps;
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
    std::vector<gca::edge> last_edges;
    for (unsigned i = 0; i < layer_depths.size(); i++) {
      auto current_edges = index_poly_edges(poly_layers[i]);
      // Front polys
      auto front_edges = symmetric_difference(current_edges, last_edges);
      auto front_polys =
	unordered_segments_to_index_polygons(front_edges);

      polys.push_back(convert_index_poly(pts, poly_layers[i], last_depth_offset, extrude_dir));


      // Sides
      concat(polys, side_polys(pts, poly_layers[i], last_depth_offset, last_depth_offset + layer_depths[i], extrude_dir));

      // Back
      if (i + 1 == layer_depths.size()) {
	oriented_polygon back =
	  convert_index_poly(pts, poly_layers[i], layer_depths[i], extrude_dir);
	vector<point> verts = back.vertices();
	reverse(begin(verts), end(verts));
	oriented_polygon back_rev(-1*extrude_dir, verts);
	polys.push_back(back_rev);
      }

      last_edges = current_edges;
      last_depth_offset += layer_depths[i];
    }
    return mesh_for_polys(polys);
  }

}

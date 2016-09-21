#include "geometry/vtk_debug.h"
#include <boost/optional.hpp>

#include "geometry/extrusion.h"
#include "utils/relation.h"

#define ANSI_DECLARATORS

#define REAL double

#include <string>
#include <stdio.h>
#include <stdlib.h>

extern "C" {
#include "triangle_lib/triangle.h"
}

namespace gca {

  oriented_polygon
  oriented_polygon_for_index_polyline(const std::vector<point>& pts,
				      const index_poly& p,
				      const point n) {
    vector<point> poly_pts;
    for (unsigned in = 0; in < p.size(); in++) {
      if (in < p.size() - 1) {
	poly_pts.push_back(pts[p[in]]);
      } else if (p[in] != p[0]) {
	poly_pts.push_back(pts[p[in]]);
      }
    }
    return oriented_polygon(n, poly_pts);
  }
  
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
    //vtk_debug_polygons(polys);
    cout << "# of polygons = " << polys.size() << endl;
    std::vector<triangle> tris;
    for (auto p : polys) {
      //vtk_debug_polygon(p);
      concat(tris, vtk_triangulate_poly(p));
    }
    //vtk_debug_triangles(tris);
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
      if (lines[i].l == points.back()) {
    	if (lines[i].r == points.front()) {
    	  lines.erase(lines.begin() + i);	  
    	  return points;
    	}
    	points.push_back(lines[i].r);
    	lines.erase(lines.begin() + i);
    	i = 0;
      } else if (lines[i].r == points.back()) {
    	if (lines[i].l == points.front()) {
    	  lines.erase(lines.begin() + i);
    	  return points;
    	}
    	points.push_back(lines[i].l);
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

  relation<index_poly, index_t>
  build_endpoint_relation(const std::vector<index_poly>& plines,
			  const std::vector<index_t>& endpoints) {
    relation<index_poly, index_t> rel(plines, endpoints);
    for (auto i : rel.left_inds()) {
      const index_poly& p = rel.left_elem(i);
      for (auto j : rel.right_inds()) {
	index_t pt = rel.right_elem(j);
	if (p.front() == pt) {
	  rel.insert(i, j);
	}
	if (p.back() == pt) {
	  rel.insert(i, j);
	}
      }
    }
    return rel;
  }

  boost::optional<index_poly>
  merge_center(const index_poly& l, const index_poly& r) {
    if (l.back() == r.front()) {
      index_poly rest(begin(r) + 1, end(r));
      index_poly lc = l;
      concat(lc, rest);
      return lc;
    }
    return boost::none;
  }
  
  boost::optional<index_poly>
  merge_adjacent(const index_poly& l, const index_poly& r) {
    auto res = merge_center(l, r);
    if (res) { return *res; }
    res = merge_center(r, l);
    if (res) { return *res; }
    index_poly lc = l;
    reverse(begin(lc), end(lc));
    res = merge_center(lc, r);
    if (res) { return *res; }
    index_poly rc = r;
    reverse(begin(rc), end(rc));
    res = merge_center(l, rc);
    if (res) { return *res; }
    return boost::none;
  }

  bool
  try_to_merge_lines(std::vector<index_poly>& plines) {
    for (unsigned i = 0; i < plines.size(); i++) {
      index_poly* pi = &(plines[i]);
      for (unsigned j = 0; j < plines.size(); j++) {
	index_poly* pj = &(plines[j]);
	if (i != j) {
	  boost::optional<index_poly> res = merge_adjacent(*pi, *pj);
	  if (res) {
	    *pi = *res;
	    plines.erase(begin(plines) + j);
	    return true;
	  }
	}
      }
    }
    return false;
  }
  
  std::vector<index_poly>
  unordered_segments_to_index_polylines(std::vector<gca::edge>& lines) {
    assert(lines.size() > 0);
    vector<index_poly> plines;
    for (auto e : lines) {
      plines.push_back({e.l, e.r});
    }
    bool merged_one = true;
    while (merged_one) {
      merged_one = try_to_merge_lines(plines);
    }
    return plines;
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
    std::vector<gca::edge> last_edges{};
    for (unsigned i = 0; i < layer_depths.size(); i++) {
      auto current_edges = index_poly_edges(poly_layers[i]);
      // Front polys
      auto front_edges = symmetric_difference(current_edges, last_edges);
      auto front_polys =
	unordered_segments_to_index_polygons(front_edges);

      for (auto p : front_polys) {
	polys.push_back(convert_index_poly(pts, p, last_depth_offset, extrude_dir));
      }


      // Sides
      concat(polys, side_polys(pts, poly_layers[i], last_depth_offset, last_depth_offset + layer_depths[i], extrude_dir));

      // Back
      if (i + 1 == layer_depths.size()) {
	// auto last_edges = index_poly_edges(poly_layers[i]);
	// auto last_polys = unordered_segments_to_index_polygons(last_edges);
	// assert(last_polys.size() == 1);
	oriented_polygon back =
	  convert_index_poly(pts, poly_layers[i], last_depth_offset + layer_depths[i], extrude_dir);
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

  triangular_mesh
  extrude(const extrusion& ext) {
    return extrude_layers(ext.pts, ext.poly_layers, ext.layer_depths, ext.extrude_dir);
  }


  // TODO: Clarify open vs. closed
  polyline
  to_polyline(const index_poly& poly,
	      const std::vector<point>& pts) {
    vector<point> res;
    for (auto i : poly) {
      res.push_back(pts[i]);
    }
    return res;
  }

  // TODO: Make this less hacky
  index_poly
  min_index_poly(const std::vector<point>& pts,
		 const std::vector<index_poly>& polys) {
    DBG_ASSERT(polys.size() > 0);
    return *(min_element(begin(polys), end(polys),
			 [pts](const index_poly& l, const index_poly& r)
			 { return pts[l.front()].z < pts[r.front()].z; }));
  }

  std::vector<gca::edge>
  index_poly_to_edges(const index_poly& p) {
    DBG_ASSERT(p.size() > 0);
    vector<gca::edge> edges;
    for (unsigned i = 0; i < p.size(); i++) {
      gca::edge e(p[i], p[(i + 1) % p.size()]);
      edges.push_back(e);
    }

    DBG_ASSERT(p.size() == edges.size());
    
    return edges;
  }

  std::vector<triangle> triangulate_flat_3d(const polygon_3& p) {
    // TODO: Handle holes
    DBG_ASSERT(p.holes().size() == 0);
    DBG_ASSERT(p.vertices().size() > 2);
    DBG_ASSERT(angle_eps(p.normal(), point(0, 0, 1), 0.0, 1.0));

    double z = p.vertices().front().z;

    struct triangulateio in, mid;

    in.numberofpoints = p.vertices().size();
    in.numberofpointattributes = 0;

    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    for (unsigned i = 0; i < p.vertices().size(); i++) {
      in.pointlist[2*i] = p.vertices()[i].x;
      in.pointlist[2*i + 1] = p.vertices()[i].y;
    }
  
    in.pointattributelist = NULL;

    in.pointmarkerlist = (int *) malloc(in.numberofpoints * sizeof(int));
    in.pointmarkerlist[0] = 0;
    in.pointmarkerlist[1] = 0;
    in.pointmarkerlist[2] = 0;
    in.pointmarkerlist[3] = 0;

    in.pointmarkerlist[4] = 0;
    in.pointmarkerlist[5] = 0;
    in.pointmarkerlist[6] = 0;
    in.pointmarkerlist[7] = 0;

    in.numberofsegments = p.vertices().size();
    /* Needed only if segments are output (-p or -c) and -P not used: */
    in.segmentlist = static_cast<int*>(malloc(2*in.numberofsegments*sizeof(int))); //(int *) NULL;

    for (unsigned i = 0; i < p.vertices().size(); i++) {
      in.segmentlist[2*i] = i;
      in.segmentlist[2*i + 1] = (i + 1) % p.vertices().size();
    }
  
    in.segmentmarkerlist = static_cast<int*>(malloc(2*in.numberofsegments*sizeof(int)));

    for (unsigned i = 0; i < in.numberofsegments; i++) {
      in.segmentmarkerlist[i] = 0;
    }

    // Need to add holes
    in.numberofholes = 0;
    in.numberofregions = 0;

    in.regionlist = NULL; // (REAL *) malloc(in.numberofregions * 4 * sizeof(REAL));

    printf("Input point set:\n\n");
    //report(&in, 1, 0, 0, 0, 0, 0);

    /* Make necessary initializations so that Triangle can return a */
    /*   triangulation in `mid' and a voronoi diagram in `vorout'.  */

    mid.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
    /* Not needed if -N switch used or number of point attributes is zero: */
    mid.pointattributelist = (REAL *) NULL;
    mid.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
    mid.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
    /* Not needed if -E switch used or number of triangle attributes is zero: */
    mid.triangleattributelist = (REAL *) NULL;
    mid.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */

    /* Needed only if segments are output (-p or -c) and -P and -B not used: */
    mid.segmentmarkerlist = (int *) NULL;
    mid.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
    mid.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */

    /* Triangulate the points.  Switches are chosen to read and write a  */
    /*   PSLG (p), preserve the convex hull (c), number everything from  */
    /*   zero (z), assign a regional attribute to each element (A), and  */
    /*   produce an edge list (e), a Voronoi diagram (v), and a triangle */
    /*   neighbor list (n).                                              */

    char settings[] = "pzen";
    triangulate(&(settings[0]), &in, &mid, NULL);

    printf("Initial triangulation:\n\n");
    //report(&mid, 1, 1, 1, 1, 1, 0);

    cout << "mid.numberofcorners = " << mid.numberofcorners << endl;

    vector<point> point_vec;
    for (unsigned i = 0; i < mid.numberofpoints; i++) {
      REAL px = mid.pointlist[2*i];
      REAL py = mid.pointlist[2*i + 1];
      point_vec.push_back(point(px, py, 0.0));
    }

    cout << "POINTS" << endl;
    for (auto p : point_vec) {
      cout << p << endl;
    }
  
    vector<triangle> tris;
    for (unsigned i = 0; i < mid.numberoftriangles; i++) {
      vector<point> pts;
      for (unsigned j = 0; j < mid.numberofcorners; j++) {
    	auto ind = mid.trianglelist[i * mid.numberofcorners + j];
    	pts.push_back(point_vec[ind]);
      }

      DBG_ASSERT(pts.size() == 3);

      point v0 = pts[0];
      point v1 = pts[1];
      point v2 = pts[2];
    
      point q1 = v1 - v0;
      point q2 = v2 - v0;
      point norm = cross(q2, q1).normalize();
      tris.push_back(triangle(norm, v0, v1, v2));
    }

    cout << "Number of triangles = " << tris.size() << endl;
    for (auto t : tris) {
      cout << t << endl;
    }

    auto pd = polydata_for_triangles(tris);
    auto act = polydata_actor(pd);
    visualize_actors({act});

    /* Free all allocated arrays, including those allocated by Triangle. */

    free(in.pointlist);
    free(in.pointattributelist);
    free(in.pointmarkerlist);
    free(in.regionlist);

    free(mid.pointlist);
    free(mid.pointattributelist);
    free(mid.pointmarkerlist);
    free(mid.trianglelist);
    free(mid.triangleattributelist);
    free(mid.trianglearealist);
    free(mid.neighborlist);
    free(mid.segmentlist);
    free(mid.segmentmarkerlist);
    free(mid.edgelist);
    free(mid.edgemarkerlist);

    return tris;
  }

  std::vector<triangle> triangulate(const polygon_3& p) {
    const rotation r = rotate_from_to(p.normal(), point(0, 0, 1));
    const rotation r_inv = inverse(r);

    polygon_3 r_p = apply(r, p);

    DBG_ASSERT(angle_eps(r_p.normal(), point(0, 0, 1), 0.0, 1.0));
    
    std::vector<triangle> flat_3d_tris = triangulate_flat_3d(r_p);

    std::vector<triangle> res;
    for (auto t : flat_3d_tris) {
      res.push_back(apply(r_inv, t));
    }

    return res;
  }

  std::vector<triangle> build_top(const std::vector<triangle>& base_tris,
				  const point v) {
    DBG_ASSERT(false);
  }

  std::vector<triangle> build_sides(const polygon_3& p, const point v) {
    DBG_ASSERT(false);
  }

  triangular_mesh extrude(const polygon_3& p, const point v) {
    DBG_ASSERT(p.holes().size() == 0);
    
    std::vector<triangle> base = triangulate(p);

    std::vector<triangle> top = build_top(base, v);
    concat(base, top);

    std::vector<triangle> sides = build_sides(p, v);
    concat(base, sides);
    
    return make_mesh(base, 0.0001);
  }
}

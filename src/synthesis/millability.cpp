#include "synthesis/millability.h"
#include "utils/algorithm.h"

namespace gca {

  std::vector<index_t> all_intersections(const vector<index_t>& all_face_inds,
					 const triangular_mesh& part,
					 const line test_segment) {
    vector<index_t> ints;
    for (auto i : all_face_inds) {
      triangle t = part.face_triangle(i);
      if (intersects(t, test_segment)) {
	ints.push_back(i);
      }
    }
    return ints;
  }

  line test_segment(const double inc,
		    const point dir,
		    point p) {
    point s = (inc*dir) + p;
    point e = ((-inc)*dir) + p;
    return line(s, e);
  }
  
  std::vector<line> construct_test_segments(const point normal,
					    const std::vector<point>& centroids,
					    const double ray_len) {
    vector<line> test_segments;
    point dir = normal.normalize();
    for (auto p : centroids) {
      line l = test_segment(ray_len, dir, p);
      test_segments.push_back(l);
    }
    return test_segments;
  }

  bool adjacent_faces(const index_t i,
		      const index_t j,
		      const triangular_mesh& m) {
    triangle_t it = m.triangle_vertices(i);
    triangle_t jt = m.triangle_vertices(j);
    for (int ii = 0; ii < 3; ii++) {
      for (int jj = 0; jj < 3; jj++) {
	if (it.v[ii] == jt.v[jj]) { return true; }
      }
    }
    return false;
  }

  bool lies_along(const point normal,
		  const triangle& t) {
    return within_eps(angle_between(t.normal, normal), 90, 2.0);
  }

  bool parallel_to(const point normal,
		   const triangle& t) {
    return within_eps(angle_between(t.normal, normal), 0, 12.0);
  }

  std::vector<index_t> initial_side_faces(const point normal,
					  const std::vector<index_t>& inds,
					  const triangular_mesh& part) {
    DBG_ASSERT(inds.size() > 0);
    vector<index_t> face_inds_left = part.face_indexes();
    subtract(face_inds_left, inds);
    vector<index_t> faces;
    vector<index_t> to_remove;
    for (auto i : face_inds_left) {
      if (lies_along(normal, part.face_triangle(i))) {
	vector<index_t> to_add;
	for (auto f : inds) {
	  if (adjacent_faces(i, f, part)) { 
	    to_add.push_back(i);
	    to_remove.push_back(i);
	    break;
	  }
	}
	concat(faces, to_add);
      }
    }
    subtract(face_inds_left, to_remove);
    return faces;
  }

  // TODO: Replace with triangular_mesh::connected_region?
  void
  collect_remaining_sides(std::vector<index_t>& side_inds,
			  const triangular_mesh& part) {
    if (side_inds.size() == 0) { return; }
    vector<index_t> face_inds_left = part.face_indexes();
    subtract(face_inds_left, side_inds);
    bool added_some;
    do {
      added_some = false;
      vector<index_t> to_remove;
      for (auto i : face_inds_left) {
	vector<index_t> to_add;
	for (auto f : side_inds) {
	  if (adjacent_faces(i, f, part) &&
	      parallel_to(part.face_orientation(f), part.face_triangle(i))) {
	    to_add.push_back(i);
	    added_some = true;
	    to_remove.push_back(i);
	    break;
	  }
	}
	concat(side_inds, to_add);
      }
      subtract(face_inds_left, to_remove);
    } while (added_some);
  }

  // TODO: Handle surfaces along the normal that are
  // several triangles deep
  std::vector<index_t> add_side_faces(const point normal,
				      std::vector<index_t>& inds,
				      const triangular_mesh& part) {
    vector<index_t> side_faces = initial_side_faces(normal, inds, part);
    collect_remaining_sides(side_faces, part);
    concat(side_faces, inds);
    return side_faces;
  }

  std::vector<index_t> top_millable_faces(const point normal,
					  const std::vector<index_t>& all_face_inds,
					  const triangular_mesh& part) {
    vector<index_t> inds;
    vector<point> centroids(all_face_inds.size());
    transform(begin(all_face_inds), end(all_face_inds),
    	      begin(centroids),
    	      [&part](const index_t i) {
    		triangle t = part.face_triangle(i);
    		return t.centroid();
    	      });

    double ray_len = 2*greater_than_diameter(normal, part.vertex_list());
    vector<line> segments = construct_test_segments(normal, centroids, ray_len);

    DBG_ASSERT(all_face_inds.size() == segments.size());
    DBG_ASSERT(centroids.size() == segments.size());
    
    for (unsigned i = 0; i < all_face_inds.size(); i++) {
      auto test_segment = segments[i];
      vector<index_t> intersecting_faces = all_intersections(all_face_inds,
    							     part,
    							     test_segment);
      if (intersecting_faces.size() > 0) {
	auto m_e = max_element(begin(intersecting_faces), end(intersecting_faces),
			       [centroids, normal](const index_t l, const index_t r) {
				 point cl = centroids[l];
				 point cr = centroids[r];
				 return signed_distance_along(cl, normal) <
				 signed_distance_along(cr, normal);
			       });
	index_t m = *m_e;
	if (m == all_face_inds[i]) {
	  inds.push_back(m);
	}
      }
    }
    return inds;
  }

  std::vector<index_t> side_millable_faces(const point normal,
					   const std::vector<index_t>& all_face_inds,
					   const triangular_mesh& part) {
    vector<index_t> vertical_faces;
    for (auto i : all_face_inds) {
      if (lies_along(normal, part.face_triangle(i))) {
	vertical_faces.push_back(i);
      }
    }
    vector<index_t> inds;

    // NOTE: Centroids adjusted along triangle normal
    vector<point> centroids(vertical_faces.size());
    transform(begin(vertical_faces), end(vertical_faces),
    	      begin(centroids),
    	      [&part](const index_t i) {
    		triangle t = part.face_triangle(i);
		// TODO: Better alternative to this ad-hoc tolerance?
    		return t.centroid() + 0.01*(t.normal.normalize());
    	      });

    double ray_len = 2*greater_than_diameter(normal, part.vertex_list());
    vector<line> segments = construct_test_segments(normal, centroids, ray_len);

    DBG_ASSERT(vertical_faces.size() == segments.size());
    DBG_ASSERT(centroids.size() == segments.size());
    
    for (unsigned i = 0; i < vertical_faces.size(); i++) {
      auto test_segment = segments[i];
      vector<index_t> intersecting_faces = all_intersections(all_face_inds,
    							     part,
    							     test_segment);
      bool found_larger = false;
      point pc = part.face_triangle(vertical_faces[i]).centroid();
      for (auto k : intersecting_faces) {
	point cl = part.face_triangle(k).centroid();
	if (signed_distance_along(cl, normal) > signed_distance_along(pc, normal)) {
	  found_larger = true;
	  break;
	}
      }
      if (!found_larger) {
	inds.push_back(vertical_faces[i]);
      }

    }
    return inds;
  }
  
  std::vector<index_t> millable_faces(const point normal,
				      const triangular_mesh& part) {
    vector<index_t> all_face_inds = part.face_indexes();
    
    vector<index_t> inds = top_millable_faces(normal, all_face_inds, part);
    vector<index_t> side_inds = side_millable_faces(normal, all_face_inds, part);
    concat(inds, side_inds);
    sort(begin(inds), end(inds));
    inds.erase(unique(begin(inds), end(inds)), end(inds));
    return inds;
  }

  std::vector<surface>
  surfaces_visible_from(const std::vector<surface>& surfaces_left,
			const point n) {
    if (surfaces_left.size() == 0) { return {}; }

    const triangular_mesh& part = surfaces_left.front().get_parent_mesh();

    std::vector<index_t> millable =
      millable_faces(n, part);
    sort(begin(millable), end(millable));

    vector<surface> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i].contained_by_sorted(millable)) {
	mill_surfaces.push_back(surfaces_left[i]);
      }
    }
    return mill_surfaces;
  }

  std::vector<index_t>
  non_prismatic_millable_faces(const point n,
			       const triangular_mesh& part) {
    vector<index_t> all_millable_faces = millable_faces(n, part);

    // Just use parallel to or orthogonal_to functions?
    vector<index_t> not_vert_or_horiz =
      select(all_millable_faces, [n, part](const index_t& s) {
	  return !(all_parallel_to({s}, part, n, 1.0) ||
		   all_orthogonal_to({s}, part, n, 1.0));
	});

    return not_vert_or_horiz;
  }

}

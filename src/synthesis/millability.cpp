#include "synthesis/millability.h"
#include "system/algorithm.h"

namespace gca {

  double signed_distance_along(const point p, const point proj_dir) {
    point dir = proj_dir.normalize();
    double len = p.dot(dir);
    return len;
  }

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

  std::vector<line> construct_test_segments(const point normal,
					    const double inc,
					    const std::vector<point>& pts) {
    vector<line> test_segments;
    point dir = normal.normalize();
    for (auto p : pts) {
      point s = (inc*dir) + p;
      point e = ((-inc)*dir) + p;
      test_segments.push_back(line(s, e));
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
    return within_eps(angle_between(t.normal, normal), 90, 0.5);
  }

  // TODO: Handle surfaces along the normal that are
  // several triangles deep
  std::vector<index_t> collect_side_faces(const point normal,
					  std::vector<index_t>& inds,
					  const triangular_mesh& part) {
    assert(inds.size() > 0);
    vector<index_t> faces = inds;
    sort(begin(faces), end(faces));
    vector<index_t> face_inds_left = part.face_indexes();
    delete_if(face_inds_left,
	      [&inds](const index_t i)
	      { return find(begin(inds), end(inds), i) != end(inds); });
    vector<index_t> to_remove;
    for (auto i : face_inds_left) {
      if (lies_along(normal, part.face_triangle(i))) {
	vector<index_t> to_add;
	for (auto f : faces) {
	  if (adjacent_faces(i, f, part)) { 
	    to_add.push_back(i);
	    to_remove.push_back(i);
	    break;
	  }
	}
	faces.insert(end(faces), begin(to_add), end(to_add));
      }
    }
    delete_if(face_inds_left,
	      [&to_remove](const index_t i)
	      { return find(begin(to_remove), end(to_remove), i) != end(to_remove); });
    return faces;
  }

  std::vector<index_t> millable_faces(const point normal,
				      const triangular_mesh& part) {
    vector<index_t> all_face_inds = part.face_indexes();
    vector<point> centroids(all_face_inds.size());
    transform(begin(all_face_inds), end(all_face_inds),
	      begin(centroids),
	      [&part](const index_t i) {
		triangle t = part.face_triangle(i);
		return t.centroid();
	      });
    vector<point> face_projections(all_face_inds.size());
    transform(begin(centroids), end(centroids),
	      begin(face_projections),
	      [normal](const point cent) {
		return project_onto(cent, normal);
	      });
    auto max_e = max_element(begin(face_projections), end(face_projections),
			     [](const point l, const point r)
			     { return l.len() < r.len(); });
    double ray_len = 2*(*max_e).len();
    vector<line> segments = construct_test_segments(normal, ray_len, centroids);
    vector<index_t> inds;
    for (auto test_segment : segments) {
      vector<index_t> intersecting_faces = all_intersections(all_face_inds,
    							     part,
    							     test_segment);
      auto m_e = max_element(begin(intersecting_faces), end(intersecting_faces),
    			     [&centroids, normal](const index_t l, const index_t r) {
    			       point cl = centroids[l];
    			       point cr = centroids[r];
    			       return signed_distance_along(cl, normal) <
    			       signed_distance_along(cr, normal);
    			     });
      index_t m = *m_e;
      inds.push_back(m);
    }
    sort(begin(inds), end(inds));
    inds.erase(unique(begin(inds), end(inds)), end(inds));
    auto res_inds = collect_side_faces(normal, inds, part);
    sort(begin(res_inds), end(res_inds));
    res_inds.erase(unique(begin(res_inds), end(res_inds)), end(res_inds));
    return res_inds;
  }

}

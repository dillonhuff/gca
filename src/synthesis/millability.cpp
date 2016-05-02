#include "synthesis/millability.h"

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
    return inds;
  }

}

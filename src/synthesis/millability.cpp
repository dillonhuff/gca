#include "synthesis/millability.h"
#include "system/algorithm.h"

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

  std::vector<line> construct_test_segments(const point normal,
					    const std::vector<point>& centroids,
					    const triangular_mesh& part) {
    double ray_len = 2*greater_than_diameter(normal, centroids);
    double inc = ray_len;
    vector<line> test_segments;
    point dir = normal.normalize();
    for (auto i : part.face_indexes()) {
      triangle t = part.face_triangle(i);
      point p = t.centroid();
      point s = (inc*dir) + p;
      point e = ((-inc)*dir) + p;
      line l(s, e);
      // if (!(intersects(t, l))) {
      // 	cout << "!(intersects(t, l))" << endl;
      // 	cout << t << endl;
      // 	cout << "DOES NOT INTERSECT LINE" << endl;
      // 	cout << l << endl;
      // 	assert(false);
      // }
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
    return within_eps(angle_between(t.normal, normal), 90, 0.5);
  }

  bool parallel_to(const point normal,
		   const triangle& t) {
    return within_eps(angle_between(t.normal, normal), 0, 12.0);
  }

  std::vector<index_t> initial_side_faces(const point normal,
					  const std::vector<index_t>& inds,
					  const triangular_mesh& part) {
    assert(inds.size() > 0);
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
    assert(side_inds.size() > 0);
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

  std::vector<index_t> millable_faces(const point normal,
				      const triangular_mesh& part) {
    cout << "Millable faces" << endl;
    vector<index_t> all_face_inds = part.face_indexes();
    vector<point> centroids(all_face_inds.size());
    transform(begin(all_face_inds), end(all_face_inds),
	      begin(centroids),
	      [&part](const index_t i) {
		triangle t = part.face_triangle(i);
		return t.centroid();
	      });
    vector<line> segments = construct_test_segments(normal, centroids, part); //construct_test_segments(normal, ray_len, centroids);
    vector<index_t> inds;
    cout << "Contructing " << segments.size() << " segments" << endl;
    for (auto test_segment : segments) {
      vector<index_t> intersecting_faces = all_intersections(all_face_inds,
    							     part,
    							     test_segment);
      // if (!(intersecting_faces.size() > 0)) {
      // 	cout << "!(intersecting_faces.size() > 0)" << endl;
      // 	cout << "Segment: " << test_segment << endl;
      // 	assert(false);
      // }
      //      cout << "# intersecting faces " << intersecting_faces.size() << endl;
      if (intersecting_faces.size() > 0) {
      // TODO: Replace with max along?	
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
    }
    cout << "Done constructing segments" << endl;
    sort(begin(inds), end(inds));
    inds.erase(unique(begin(inds), end(inds)), end(inds));
    auto res_inds = add_side_faces(normal, inds, part);
    sort(begin(res_inds), end(res_inds));
    res_inds.erase(unique(begin(res_inds), end(res_inds)), end(res_inds));
    cout << "DONE millable_faces" << endl;
    return res_inds;
  }

}

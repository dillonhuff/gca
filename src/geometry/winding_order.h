#pragma once

namespace gca {

  template<typename Triangle>
  bool winding_conflict(const Triangle ti, const Triangle tj) {
    for (unsigned l = 0; l < 3; l++) {
      unsigned lp1 = (l + 1) % 3;
      for (unsigned k = 0; k < 3; k++) {
	unsigned kp1 = (k + 1) % 3;

	if (get_vertex(ti, k) == get_vertex(tj, l) &&
	    get_vertex(ti, kp1) == get_vertex(tj, lp1)) {
	  return true;
	}

      }
    }
    return false;
  }

  template<typename Triangle>
  Triangle flip_winding_order(const Triangle& t) {
    Triangle f;
    set_vertex(f, 0, get_vertex(t, 1));
    set_vertex(f, 1, get_vertex(t, 0));
    set_vertex(f, 2, get_vertex(t, 2));
    
    return f;
  }

  template<typename Triangle>
  std::vector<Triangle>
  flip_winding_orders(const std::vector<Triangle>& vertex_triangles) {
    vector<Triangle> tris;
    for (auto t : vertex_triangles) {
      tris.push_back(flip_winding_order(t));
    }
    return tris;
  }

  template<typename Triangle>
  Triangle
  correct_orientation(const Triangle to_correct,
		      const std::vector<Triangle>& others) {
    auto ti = to_correct;
    for (auto tj : others) {
      if (winding_conflict(ti, tj)) {
	Triangle corrected = flip_winding_order(ti);

	return corrected;
      }

    }

    return ti;
  }

  template<typename Triangle>
  int
  num_winding_order_errors(const std::vector<Triangle>& triangles) {
    int num_errs = 0;
    for (unsigned i = 0; i < triangles.size(); i++) {
      for (unsigned j = i; j < triangles.size(); j++) {
	if (i != j) {
	  auto ti = triangles[i];
	  auto tj = triangles[j];

	  if (winding_conflict(ti, tj)) {
	    num_errs++;
	  }
	  
	}
      }
    }
    return num_errs;
  }

  
}

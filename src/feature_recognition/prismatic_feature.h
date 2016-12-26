#pragma once

#include "geometry/polygon_3.h"
#include "geometry/homogeneous_transformation.h"
#include "geometry/rotation.h"

namespace gca {

  class feature {
  protected:
    bool closed;
    bool through;
    polygon_3 base_poly;
    double dp;
    
  public:
    feature(const bool p_closed,
	    const bool p_through,
	    const double p_depth,
	    const polygon_3& p_base) :
      closed(p_closed), through(p_through), base_poly(p_base), dp(p_depth) {
      DBG_ASSERT(depth() >= 0);
    }

    double min_distance_along(const point n) const {
      vector<point> pts;
      for (auto p : base_poly.vertices()) {
	pts.push_back(p);
	pts.push_back(p + dp*base_poly.normal());
      }

      return gca::min_distance_along(pts, n);
    }

    bool is_through() const { return through; }

    double max_distance_along(const point n) const {
      vector<point> pts;
      for (auto p : base_poly.vertices()) {
	pts.push_back(p);
	pts.push_back(p + dp*base_poly.normal());
      }

      return gca::max_distance_along(pts, n);
    }
    
    std::pair<double, double> range_along(const point n) const {
      double min = min_distance_along(n);
      double max = max_distance_along(n);
      return std::make_pair(min, max);
    }

    double depth() const { return dp; }

    polygon_3 base() const { return base_poly; }

    polygon_3 top() const {
      vector<point> verts;

      for (auto p : base_poly.vertices()) {
	verts.push_back(p + dp*base_poly.normal());
      }

      cout << "# of verts = " << verts.size() << endl;

      vector<vector<point>> holes;
      for (auto h : base_poly.holes()) {
	vector<point> hole_pts;
	for (auto p : h) {
	  hole_pts.push_back(p + dp*base_poly.normal());
	}
	holes.push_back(hole_pts);
      }

      polygon_3 top =
	build_clean_polygon_3(verts, holes);

      top.correct_winding_order(base().normal());

      return top;
    }

    double base_distance_along_normal() const
    { return gca::min_distance_along(base_poly.vertices(), base_poly.normal()); }

    feature apply(const rotation& r) const {
      polygon_3 rotated_base = gca::apply(r, base_poly);
      rotated_base.correct_winding_order(times_3(r, base_poly.normal()));

      return feature(is_closed(), is_through(), dp, rotated_base);
    }

    bool is_closed() const { return closed; }

    void set_closed(const bool cl) { closed = cl; }

    feature apply(const homogeneous_transform& t) const {
      labeled_polygon_3 rotated_base = gca::apply(t, base_poly);
      rotated_base.correct_winding_order(times_3(t.first, base_poly.normal()));

      return feature(is_closed(), is_through(), dp, rotated_base);
    }

    point normal() const { return base_poly.normal(); }
    
  };


  bool contains(const feature& maybe_contained,
		const std::vector<feature*>& container);

  std::vector<feature*>
  containing_subset(const feature& maybe_contained,
		    const std::vector<feature*>& container);

  bool same_base(const feature& l, const feature& r, const double tol);

  double base_area(const feature& f);

  bool is_outer(const feature& f, const polygon_3& stock_bound);  

}

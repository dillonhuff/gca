#include "feature_recognition/prismatic_feature.h"

namespace gca {

  std::ostream& operator<<(std::ostream& out, const std::pair<double, double>& r) {
    out << "( " << r.first << " , " << r.second << " )" << endl;
    return out;
  }
  
  bool contains(const feature& maybe_contained,
		const std::vector<feature*>& container) {
    for (auto c : container) {
      DBG_ASSERT(angle_eps(maybe_contained.normal(), c->normal(), 180.0, 1.0));
    }

    vector<feature*> sub =
      containing_subset(maybe_contained, container);

    return sub.size() > 0;
  }
 
  bool same_base(const feature& l, const feature& r, const double tol) {
    auto l_p = l.base();
    auto r_p = r.base();

    const rotation rot = rotate_from_to(l_p.normal(), point(0, 0, 1));

    auto l_pr = to_boost_poly_2(apply(rot, l_p));
    auto r_pr = to_boost_poly_2(apply(rot, r_p));

    boost_multipoly_2 res;
    boost::geometry::sym_difference(l_pr, r_pr, res);

    return boost::geometry::area(res) < tol;
  }

  double base_area(const feature& f) {
    return area(f.base());
  }

  bool is_outer(const feature& f, const polygon_3& stock_bound) {
    const rotation r = rotate_from_to(f.normal(), point(0, 0, 1));
    polygon_3 base_p =
      build_clean_polygon_3(f.base().vertices());
    auto base_poly = to_boost_poly_2(apply(r, base_p));
    auto stock_poly = to_boost_poly_2(apply(r, stock_bound));

    boost_multipoly_2 sym_diff;

    bg::sym_difference(base_poly, stock_poly, sym_diff);
    if (bg::area(sym_diff) < 0.001) {

#ifdef VIZ_DBG      
      cout << "OUTER FEATURE" << endl;
      vtk_debug_feature(f);
#endif

      return true;
    }

    return false;
  }

  bool adjacent(const std::pair<double, double> r1,
		const std::pair<double, double> r2,
		const double tol) {
    if ((r1.first < r1.second) &&
	(r2.first < r2.second)) {
      if (within_eps(r1.second, r2.first, tol)) { return true; }
      if (within_eps(r2.second, r1.first, tol)) { return true; }
    } else {
      cout << "r1 = " << r1 << endl;
      cout << "r2 = " << r2 << endl;
      DBG_ASSERT(false);
    }

    return false;
  }

  std::vector<feature*>
  range_containing(const point n,
		   const feature& maybe_contained,
		   const std::vector<feature*>& maybe_container) {

    auto contained_range = maybe_contained.range_along(n);

    vector<feature*> container;
    for (auto f : maybe_container) {
      auto f_range = f->range_along(n);
      if (intervals_overlap(f_range, contained_range)) {

	// Do not include features that just border the feature
	// in the containing range
	if (!adjacent(f_range, contained_range, 0.001)) {
	  container.push_back(f);
	}
      }
    }

    return container;
  }

  // TODO: Should really be containing outlines
  std::vector<feature*>
  overlapping_outlines(const feature& maybe_contained,
		       const std::vector<feature*>& container) {
    vector<feature*> overlapping;

    point n = maybe_contained.normal();

    const rotation r = rotate_from_to(n, point(0, 0, 1));

    labeled_polygon_3 rmc = apply(r, maybe_contained.base());
    auto rmcbp = to_boost_poly_2(rmc);
    //    double rmcbp_area = boost::g::area(rmcbp);
    
    for (auto c : container) {
      DBG_ASSERT(c != nullptr);

      labeled_polygon_3 rc = apply(r, c->base());
      rc.correct_winding_order(point(0, 0, 1));

      auto rcbp = to_boost_poly_2(rc);

      if (boost::geometry::within(rmcbp, rcbp)) {
	overlapping.push_back(c);
      } else {
	boost_multipoly_2 res;
	boost::geometry::sym_difference(rmcbp, rcbp, res);
      
	double not_shared_area = boost::geometry::area(res);

	if (not_shared_area < 0.001) {
	  overlapping.push_back(c);
	}
      }
    }

    cout << "# of overlapping = " << overlapping.size() << endl;

    return overlapping;
  }
  
  std::vector<feature*>
  containing_subset(const feature& maybe_contained,
		    const std::vector<feature*>& container) {
    cout << "Container size = " << container.size() << endl;
    cout << "maybe_contained normal = " << maybe_contained.normal() << endl;
    cout << "Start checking normals" << endl;
    for (auto c : container) {
      DBG_ASSERT(angle_eps(maybe_contained.normal(), c->normal(), 180.0, 1.0) ||
		 angle_eps(maybe_contained.normal(), c->normal(), 0.0, 1.0));
    }
    cout << "DONE CHECKING NORMALS" << endl;

    vector<feature*> outlines = overlapping_outlines(maybe_contained, container);

    point n = maybe_contained.normal();
    return range_containing(n, maybe_contained, outlines);
  }

}

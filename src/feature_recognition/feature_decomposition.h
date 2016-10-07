#ifndef GCA_FEATURE_DECOMPOSITION_H
#define GCA_FEATURE_DECOMPOSITION_H

#include <deque>

#include "geometry/polygon_3.h"
#include "synthesis/contour_planning.h"

namespace gca {

  class feature {
  protected:
    labeled_polygon_3 base_poly;
    double dp;
    
  public:
    feature(const double p_depth,
	    const labeled_polygon_3& p_base) :
      base_poly(p_base), dp(p_depth) {
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

    labeled_polygon_3 base() const { return base_poly; }

    labeled_polygon_3 top() const {
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

      labeled_polygon_3 top(verts, holes);

      top.correct_winding_order(base().normal());

      return top;
    }

    double base_distance_along_normal() const
    { return gca::min_distance_along(base_poly.vertices(), base_poly.normal()); }

    feature apply(const rotation& r) const {
      labeled_polygon_3 rotated_base = gca::apply(r, base_poly);
      rotated_base.correct_winding_order(times_3(r, base_poly.normal()));

      return feature(dp, rotated_base);
    }

    feature apply(const homogeneous_transform& t) const {
      labeled_polygon_3 rotated_base = gca::apply(t, base_poly);
      rotated_base.correct_winding_order(times_3(t.first, base_poly.normal()));

      return feature(dp, rotated_base);
    }

    point normal() const { return base_poly.normal(); }
    
  };

  class feature_decomposition {
  protected:
    feature* feat;

    std::vector<feature_decomposition*> children;

  public:
    feature_decomposition() :
      feat(nullptr) {}

    feature_decomposition(feature* p_feat) :
      feat(p_feat) {}

    const gca::feature* feature() const { return feat; }
    gca::feature* feature() { return feat; }
    
    void add_child(feature_decomposition* decomp)
    { children.push_back(decomp); }

    void delete_child(const unsigned i)
    { children.erase(begin(children) + i); }

    void set_feature(class feature* f) {
      feat = f;
    }

    unsigned num_children() const { return children.size(); }

    feature_decomposition* child(unsigned i) const {
      DBG_ASSERT(i < children.size());
      return children[i];
    }

    unsigned num_features() const {
      unsigned num_child_features = 0;
      for (auto c : children) {
	num_child_features += c->num_features();
      }
      return (feat == nullptr ? 0 : 1) + num_child_features;
    }

    unsigned num_levels() const {
      if (children.size() > 0) {
	unsigned max_child_num_levels =
	  max_e(children,
		[](const feature_decomposition* d) { return d->num_levels(); })->num_levels();
	return 1 + max_child_num_levels;
      }

      return 1;
    }
  };

  gca::feature* node_value(feature_decomposition* f);

  template<typename T, typename F>
  void delete_leaves(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;

      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (next->num_children() == 0 && should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_leaves(tree->child(i), should_delete);
    }
  }

  template<typename T, typename F>
  void delete_internal_nodes(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;
      
      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (next->num_children() != 0 && should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_internal_nodes(tree->child(i), should_delete);
    }
  }
  
  template<typename T, typename F>
  void delete_nodes(T* tree, F should_delete) {
    bool deleted_one = true;

    while (deleted_one) {
      deleted_one = false;
      
      for (unsigned i = 0; i < tree->num_children(); i++) {
	T* next = tree->child(i);
	if (should_delete(node_value(next))) {
	  tree->delete_child(i);
	  deleted_one = true;
	  break;
	}
      }

    }

    for (unsigned i = 0; i < tree->num_children(); i++) {
      delete_nodes(tree->child(i), should_delete);
    }
  }
  
  template<typename T, typename F>
  void traverse_bf(T* tree, F f) {
    std::deque<T*> active{tree};
    while (active.size() > 0) {
      T* next = active.front();
      f(node_value(next));

      active.pop_front();

      for (auto i = 0; i < next->num_children(); i++) {
	active.push_back(next->child(i));
      }
    }
  }

  template<typename T, typename F>
  void traverse_leaves_bf(T* tree, F f) {
    std::deque<T*> active{tree};
    while (active.size() > 0) {
      T* next = active.front();

      if (next->num_children() == 0) {
	f(node_value(next));
      }

      active.pop_front();

      for (auto i = 0; i < next->num_children(); i++) {
	active.push_back(next->child(i));
      }
    }
  }

  labeled_polygon_3
  to_labeled_polygon_3(const rotation& r, const double z, const boost_poly_2& p);

  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p);
  
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n);

  vector<feature*> collect_features(feature_decomposition* f);

  surface_levels
  initial_surface_levels(const triangular_mesh& m,
			 const point n);

  point normal(feature_decomposition* f);

  bool contains(const feature& maybe_contained,
		const std::vector<feature*>& container);

  std::vector<feature*>
  containing_subset(const feature& maybe_contained,
		    const std::vector<feature*>& container);

  std::vector<feature*> collect_leaf_features(feature_decomposition* f);

  bool same_base(const feature& l, const feature& r, const double tol);

  feature* parent_feature(feature* f, feature_decomposition* decomp);

  double base_area(const feature& f);

  labeled_polygon_3 shrink(const labeled_polygon_3& p, const double tol);

  boost::optional<labeled_polygon_3>
  shrink_optional(const labeled_polygon_3& p,
		  const double tol);

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& stock,
			      const triangular_mesh& m,
			      const point n);


}
#endif

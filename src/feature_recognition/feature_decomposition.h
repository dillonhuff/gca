#ifndef GCA_FEATURE_DECOMPOSITION_H
#define GCA_FEATURE_DECOMPOSITION_H

#include <deque>

#include "geometry/rotation.h"
#include "synthesis/contour_planning.h"

namespace gca {

  template<typename Ring>
  point ring_normal(const Ring& r) {
    point l1 = r[1] - r[0];
    point l2 = r[2] - r[0];
    return cross(l1, l2).normalize();
  }

  template<typename Ring>
  void correct_winding_order(Ring& r, const point n) {
    double theta = angle_between(ring_normal(r), n);
    if (within_eps(theta, 0, 0.1)) { return; }

    cout << "Ring normal before = " << ring_normal(r) << endl;
      
    DBG_ASSERT(within_eps(theta, 180, 0.1));

    reverse(r);

    cout << "Ring normal after = " << ring_normal(r) << endl;

    double new_theta = angle_between(ring_normal(r), n);

    DBG_ASSERT(within_eps(new_theta, 0, 0.1));
  }
  
  class labeled_polygon_3 {
  protected:
    std::vector<point> outer_ring;
    std::vector<std::vector<point>> inner_rings;
    
  public:
    labeled_polygon_3(const std::vector<point> vertices) :
      outer_ring(vertices) {
      DBG_ASSERT(outer_ring.size() >= 3);
    }

    labeled_polygon_3(const std::vector<point> vertices,
		      const std::vector<std::vector<point>> hole_verts) :
      outer_ring(vertices),
      inner_rings(hole_verts) {
      DBG_ASSERT(outer_ring.size() >= 3);
    }
    
    point normal() const {
      return ring_normal(outer_ring);
    }

    point vertex(const unsigned i) const { return outer_ring[i]; }

    const std::vector<point>& vertices() const { return outer_ring; }

    const std::vector<std::vector<point>>& holes() const { return inner_rings; }
    
    unsigned num_vertices() const { return outer_ring.size(); }

    void correct_winding_order(const point n) {
      gca::correct_winding_order(outer_ring, n);
      for (std::vector<point>& ir : inner_rings) {
	gca::correct_winding_order(ir, n);
      }
    }
  };

  labeled_polygon_3 apply(const rotation& r, const labeled_polygon_3& p);

  typedef std::vector<std::vector<labeled_polygon_3>> surface_levels;

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

    double depth() const { return dp; }

    labeled_polygon_3 base() const { return base_poly; }

    feature apply(const rotation& r) const {
      return feature(dp, gca::apply(r, base_poly));
    }
    
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

  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p);
  
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n);

  vector<feature*> collect_features(feature_decomposition* f);
}

#endif

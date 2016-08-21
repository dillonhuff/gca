#ifndef GCA_FEATURE_DECOMPOSITION_H
#define GCA_FEATURE_DECOMPOSITION_H

#include "synthesis/contour_planning.h"

namespace gca {

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
      point l1 = outer_ring[1] - outer_ring[0];
      point l2 = outer_ring[2] - outer_ring[0];
      return cross(l1, l2).normalize();
    }

    point vertex(const unsigned i) const { return outer_ring[i]; }

    const std::vector<point>& vertices() const { return outer_ring; }

    const std::vector<std::vector<point>>& holes() const { return inner_rings; }
    
    unsigned num_vertices() const { return outer_ring.size(); }
  };

  typedef std::vector<std::vector<labeled_polygon_3>> surface_levels;

  class feature {};

  class feature_decomposition {
  protected:
    feature feat;

    std::vector<feature_decomposition*> children;

  public:
    void add_child(feature_decomposition* decomp)
    { children.push_back(decomp); }

    unsigned num_features() const {
      unsigned num_child_features = 0;
      for (auto c : children) {
	num_child_features += c->num_features();
      }
      return 1 + num_child_features;
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

  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n);

}

#endif

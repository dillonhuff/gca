#ifndef GCA_FIXTURE_ANALYSIS_H
#define GCA_FIXTURE_ANALYSIS_H

#include "geometry/surface.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece.h"

namespace gca {

  typedef std::map<unsigned, std::vector<unsigned>> orientation_map;
  typedef std::map<unsigned, std::vector<unsigned>> surface_map;

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces);

  class stock_orientation {
  protected:
    const surface* left;
    const surface* right;
    const surface* bottom;

  public:

    inline const surface& get_left() const { return *left; }
    inline const surface& get_right() const { return *right; }
    inline const surface& get_bottom() const { return *bottom; }
    
    inline point top_normal() const {
      point bn = bottom->face_orientation(bottom->front());
      point n = bn - 2*bn;
      assert(within_eps(angle_between(n, bn), 180, 0.1));
      return n;
    }

    inline point left_normal() const {
      point bn = left->face_orientation(bottom->front());
      point n = bn - 2*bn;
      assert(within_eps(angle_between(n, bn), 180, 0.1));
      return n;
    }
    
    inline const triangular_mesh& get_mesh() const
    { return left->get_parent_mesh(); }

    stock_orientation(const surface* p_left,
		      const surface* p_right,
		      const surface* p_bottom) :
      left(p_left), right(p_right), bottom(p_bottom) {}

    stock_orientation() :
      left(nullptr), right(nullptr), bottom(nullptr) {}
  };
  
  typedef std::vector<std::vector<index_t>> surface_list;

  std::vector<surface> outer_surfaces(const triangular_mesh& part);

  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w);

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh);

  std::vector<std::pair<stock_orientation, surface_list>>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& stable_surfaces,
		      const vice& v);

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part_mesh,
				       const std::vector<surface>& stable_surfaces);

  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces);

  // TODO: Should this really be a map?
  surface_map
  pick_orientations(const triangular_mesh& part_mesh,
		    const std::vector<surface>& surfaces_to_cut,
		    std::vector<stock_orientation>& all_orients,
		    const vice& v);
  
}

#endif

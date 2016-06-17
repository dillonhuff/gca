#ifndef GCA_FIXTURE_ANALYSIS_H
#define GCA_FIXTURE_ANALYSIS_H

#include "geometry/surface.h"
#include "synthesis/tool.h"
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

    inline point bottom_plane_point() const {
      const triangular_mesh& m = bottom->get_parent_mesh();
      triangle_t t = m.triangle_vertices(bottom->front());
      return m.vertex(t.v[0]);
    }
    
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

  struct fixture {
    stock_orientation orient;
    vice v;
    fixture(const stock_orientation& p_orient,
	    const vice& p_v)
      : orient(p_orient), v(p_v) {}
  };

  class fixtures {
  protected:
    vice v;
    std::vector<plate_height> plates;
    
  public:

    fixtures(const vice& p_v)
      : v(p_v) {}
    
    fixtures(const vice& p_v, const std::vector<plate_height>& p_plates)
      : v(p_v), plates(p_plates) {}
    
    inline const vice& get_vice() const { return v; }
    inline const std::vector<plate_height>& base_plates() const { return plates; }
  };

  typedef std::vector<std::pair<fixture, surface_list>> fixture_list;

  class fixture_plan {
  protected:
    const triangular_mesh& part;
    workpiece stock;
    fixture_list fixture_pairs;
    
  public:
    fixture_plan(const triangular_mesh& p_part,
		 const workpiece& p_stock,
		 const fixture_list& p_fixture_pairs) :
      part(p_part), stock(p_stock), fixture_pairs(p_fixture_pairs) {
      std::cout << "Fixture plan: # of pairs = " << fixture_pairs.size() << endl;
    }

    workpiece aligned_workpiece() const
    { return stock; }

    const fixture_list& fixtures() const
    { return fixture_pairs; }
  };
  
  std::vector<surface> outer_surfaces(const triangular_mesh& part);

  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w);

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh);

  std::pair<std::vector<surface>, fixture_list>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& stable_surfaces,
		      const fixtures& v);

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part_mesh,
				       const std::vector<surface>& stable_surfaces);

  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces,
			  const vice& v);

  std::vector<fixture>
  all_stable_fixtures(const std::vector<surface>& surfaces,
		      const fixtures& v);
  
  // TODO: Should this really be a map?
  surface_map
  pick_orientations(const triangular_mesh& part_mesh,
		    const std::vector<surface>& surfaces_to_cut,
		    std::vector<fixture>& all_orients,
		    const vice& v);
  

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 std::vector<surface>& part_ss,
				 const fixtures& fixes,
				 const vector<tool>& tools,
				 const workpiece w);

}

#endif

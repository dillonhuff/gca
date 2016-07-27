#ifndef GCA_FIXTURE_ANALYSIS_H
#define GCA_FIXTURE_ANALYSIS_H

#include "geometry/plane.h"
#include "geometry/surface.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece.h"

namespace gca {

  typedef std::map<unsigned, std::vector<unsigned>> surface_map;

  struct fixture {
    clamp_orientation orient;
    vice v;
    fixture(const clamp_orientation& p_orient,
	    const vice& p_v)
      : orient(p_orient), v(p_v) {}
  };

  class fixtures {
  protected:
    vice v;
    std::vector<plate_height> par_plates;
    
  public:

    fixtures(const vice& p_v)
      : v(p_v) {}
    
    fixtures(const vice& p_v,
	     const std::vector<plate_height>& p_par_plates)
      : v(p_v), par_plates(p_par_plates) {}
    
    inline const vice& get_vice() const { return v; }
    inline const std::vector<plate_height>& parallel_plates() const
    { return par_plates; }
  };

  struct fixture_setup {
    const triangular_mesh* m;
    fixture fix;
    std::vector<pocket> pockets;

    fixture_setup(const triangular_mesh* p_m,
		  const fixture& f,
		  const std::vector<pocket>& p)
      : m(p_m), fix(f), pockets(p) {}
  };

  class fixture_plan {
  protected:
    const triangular_mesh& part;
    std::vector<fixture_setup> setups;
    
  public:
    fixture_plan(const triangular_mesh& p_part,
		 const std::vector<fixture_setup>& p_fixture_pairs) :
      part(p_part), setups(p_fixture_pairs) {
      std::cout << "Fixture plan: # of setups = " << setups.size() << endl;
    }

    const std::vector<fixture_setup>& fixtures() const
    { return setups; }
  };

  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece& w);

  std::vector<surface> surfaces_to_cut(const triangular_mesh& part_mesh,
				       const std::vector<surface>& stable_surfaces);
  std::vector<fixture_setup>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfs_to_cut,
		      const fixtures& v);

  std::vector<fixture>
  all_stable_fixtures(const std::vector<surface>& surfaces,
		      const fixtures& v);

  fixture_plan make_fixture_plan(const triangular_mesh& part_mesh,
				 const fixtures& fixes,
				 const vector<tool>& tools,
				 const workpiece w);

  std::vector<pocket> make_surface_pockets(const triangular_mesh& mesh,
					   std::vector<std::vector<index_t>>& surfaces);

  std::vector<surface>
  stable_surfaces_after_clipping(const triangular_mesh& part_mesh,
				 const triangular_mesh& aligned_workpiece_mesh);

}

#endif

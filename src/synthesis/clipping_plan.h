#ifndef GCA_CLIPPING_PLAN_H
#define GCA_CLIPPING_PLAN_H

#include "synthesis/clamp_orientation.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

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

  class clipping_plan {
  public:
    std::vector<surface> stable_surfs;
    std::vector<surface> surfs_to_cut;
    std::vector<fixture_setup> fixtures;

    clipping_plan(const std::vector<surface>& p_stable_surfs,
		  const std::vector<surface>& p_surfs_to_cut,
		  const std::vector<fixture_setup>& p_fixtures)
      : stable_surfs(p_stable_surfs),
	surfs_to_cut(p_surfs_to_cut),
	fixtures(p_fixtures) {}

    inline const std::vector<surface>& stable_surfaces() const
    { return stable_surfs; }

    inline const std::vector<surface>& surfaces_left_to_cut() const
    { return surfs_to_cut; }

  };

}

#endif

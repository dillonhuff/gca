#ifndef GCA_CLIPPING_PLAN_H
#define GCA_CLIPPING_PLAN_H

#include "geometry/rigid_arrangement.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/fabrication_plan.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  struct fixture {
    clamp_orientation orient;
    vice v;
    fixture(const clamp_orientation& p_orient,
	    const vice& p_v)
      : orient(p_orient), v(p_v) {}
  };

  class fixture_setup {
  protected:
    rigid_arrangement a;

  public:
    fixture fix;
    std::vector<pocket> pockets;

    fixture_setup(const triangular_mesh* p_m,
		  const fixture& f,
		  const std::vector<pocket>& p)
      : fix(f), pockets(p) {
      a.insert("part", *p_m);
    }

    fixture_setup(const rigid_arrangement& p_a,
		  const fixture& f,
		  const std::vector<pocket>& p)
      : a(p_a), fix(f), pockets(p) {}
    
    const triangular_mesh& part_mesh() const { return a.mesh("part"); }
    const rigid_arrangement& arrangement() const { return a; }

  };

  class clipping_plan {
  public:
    std::vector<surface> stable_surfs;
    std::vector<surface> surfs_to_cut;
    std::vector<fixture_setup> fixtures;
    std::vector<fabrication_plan*> custom_fixes;

    clipping_plan(const std::vector<surface>& p_stable_surfs,
		  const std::vector<surface>& p_surfs_to_cut,
		  const std::vector<fixture_setup>& p_fixtures)
      : stable_surfs(p_stable_surfs),
	surfs_to_cut(p_surfs_to_cut),
	fixtures(p_fixtures) {}

    clipping_plan(const std::vector<surface>& p_stable_surfs,
		  const std::vector<surface>& p_surfs_to_cut,
		  const std::vector<fixture_setup>& p_fixtures,
		  const std::vector<fabrication_plan*>& p_custom_fixes)
      : stable_surfs(p_stable_surfs),
	surfs_to_cut(p_surfs_to_cut),
	fixtures(p_fixtures),
	custom_fixes(p_custom_fixes) {}
    
    inline const std::vector<surface>& stable_surfaces() const
    { return stable_surfs; }

    inline const std::vector<surface>& surfaces_left_to_cut() const
    { return surfs_to_cut; }

    const std::vector<fabrication_plan*>& custom_fixtures() const
    { return custom_fixes; }
    
  };

}

#endif

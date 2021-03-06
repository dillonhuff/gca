#ifndef GCA_CLIPPING_PLAN_H
#define GCA_CLIPPING_PLAN_H

#include "backend/operation.h"
#include "geometry/rigid_arrangement.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/fabrication_plan.h"

namespace gca {

  class fixture {
  protected:
    boost::optional<clamp_orientation> zero_planes;

  public:
    clamp_orientation orient;
    vice v;
    fixture(const clamp_orientation& p_orient,
	    const vice& p_v)
      : zero_planes(boost::none), orient(p_orient), v(p_v) {}

    fixture(const clamp_orientation& p_orient,
	    const vice& p_v,
	    const clamp_orientation& p_zero_planes)
      : zero_planes(p_zero_planes), orient(p_orient), v(p_v) {}

    bool has_part_zero() const {
      if (zero_planes) { return true; }
      return false;
    }

    clamp_orientation get_zero_planes() const {
      DBG_ASSERT(zero_planes);

      return *zero_planes;
    }
    
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
    workpiece w;

    clipping_plan(const std::vector<surface>& p_stable_surfs,
		  const std::vector<surface>& p_surfs_to_cut,
		  const std::vector<fixture_setup>& p_fixtures,
		  const workpiece p_w)
      : stable_surfs(p_stable_surfs),
	surfs_to_cut(p_surfs_to_cut),
	fixtures(p_fixtures),
	w(p_w) {}

    clipping_plan(const std::vector<surface>& p_stable_surfs,
		  const std::vector<surface>& p_surfs_to_cut,
		  const std::vector<fixture_setup>& p_fixtures,
		  const std::vector<fabrication_plan*>& p_custom_fixes,
		  const workpiece p_w)
      : stable_surfs(p_stable_surfs),
	surfs_to_cut(p_surfs_to_cut),
	fixtures(p_fixtures),
	custom_fixes(p_custom_fixes),
	w(p_w) {}
    
    inline const std::vector<surface>& stable_surfaces() const
    { return stable_surfs; }

    inline const std::vector<surface>& surfaces_left_to_cut() const
    { return surfs_to_cut; }

    const std::vector<fabrication_plan*>& custom_fixtures() const
    { return custom_fixes; }

    workpiece stock() const { return w; }
    
  };

}

#endif

#ifndef GCA_WORKPIECE_CLIPPING_H
#define GCA_WORKPIECE_CLIPPING_H

#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece.h"

namespace gca {

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
  
  triangular_mesh stock_mesh(const workpiece& w);
  
  //  std::pair<triangular_mesh, std::vector<fixture_setup> >
  clipping_plan
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& fixes);


  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f);
  
}

#endif

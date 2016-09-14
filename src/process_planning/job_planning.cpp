#include "process_planning/job_planning.h"
#include "utils/check.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
    vector<surface> surfs = outer_surfaces(aligned_workpiece);

    DBG_ASSERT(surfs.size() == 6);

    vector<surface> ax_surfaces =
      take_basis(surfs,
		 [](const surface& l, const surface& r)
		 { return within_eps(angle_between(normal(l), normal(r)), 90, 2.0); },
		 3);
    
    vector<point> norms;
    for (auto ax : ax_surfaces) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
      norms.push_back(-1*n);
    }

    vice v = f.get_vice();
    triangular_mesh current_stock = aligned_workpiece;
    vector<fixture_setup> cut_setups;    
    for (auto n : norms) {
      auto sfs = outer_surfaces(current_stock);

      DBG_ASSERT(sfs.size() == 6);

      auto orients = all_stable_orientations(sfs, v);
      clamp_orientation orient = find_orientation_by_normal(orients, n);

      auto t = mating_transform(current_stock, orient, v);
      cut_setups.push_back(clip_base(apply(t, current_stock), apply(t, part_mesh), fixture(orient, v)));
      
      plane clip_plane = face_plane(part_mesh, n);
      current_stock = clip_mesh(current_stock, clip_plane);
    }

    DBG_ASSERT(cut_setups.size() == 6);

    return cut_setups;
  }
  
}

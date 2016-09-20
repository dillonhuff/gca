#include "geometry/mesh_operations.h"
#include "process_planning/job_planning.h"
#include "utils/check.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
    vector<surface> surfs = outer_surfaces(stock);

    DBG_ASSERT(surfs.size() == 6);

    vector<point> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
    }

    DBG_ASSERT(norms.size() == 6);

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      feature_decomposition* decomp = build_feature_decomposition(stock, part, n);
      tool_access_info info = find_accessable_tools(decomp, tools);
      dir_info.push_back({decomp, info});
    }

    vice v = f.get_vice();
    triangular_mesh current_stock = stock;
    vector<fixture_setup> cut_setups;

    for (auto& info : dir_info) {
      auto sfs = outer_surfaces(current_stock);
      auto orients = all_stable_orientations(sfs, v);

      point n = normal(info.decomp);

      clamp_orientation orient = find_orientation_by_normal(orients, n);



      // // auto t = mating_transform(current_stock, orient, v);
      // // cut_setups.push_back(clip_base(apply(t, current_stock), apply(t, part), fixture(orient, v)));
      
      // plane clip_plane = face_plane(part, n);
      // current_stock = clip_mesh(current_stock, clip_plane);
    }

    return cut_setups;
  }
  
}

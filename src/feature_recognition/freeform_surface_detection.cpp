#include "feature_recognition/chamfer_detection.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "synthesis/millability.h"

namespace gca {

  std::vector<freeform_surface>
  freeform_surface_regions(const triangular_mesh& part,
			   const point n,
			   const std::vector<tool>& tools) {
    
    vector<index_t> all_inds =
      non_prismatic_millable_faces(n, part);

    vector<chamfer> crs = chamfer_regions(part, n, tools);
    for (auto& chamfer : crs) {
      subtract(all_inds, chamfer.faces);
    }

    if (all_inds.size() == 0) { return {}; }

    auto regions = connect_regions(all_inds, part);

    vector<freeform_surface> surfs;
    for (auto& r : regions) {
      surface s(&part, r);
      //vtk_debug_highlight_inds(s);

      vector<tool> ball_tools =
	select(tools, [](const tool& t) { return t.type() == BALL_NOSE; });

      freeform_surface surf{s, ball_tools};
      surfs.push_back(surf);
    }

    return surfs;
  }

}

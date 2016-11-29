#include "feature_recognition/chamfer_detection.h"
#include "synthesis/millability.h"

namespace gca {

  std::vector<index_t>
  non_prismatic_millable_faces(const point n,
			       const triangular_mesh& part) {
    vector<index_t> all_millable_faces = millable_faces(n, part);

    // Just use parallel to or orthogonal_to functions?
    vector<index_t> not_vert_or_horiz =
      select(all_millable_faces, [n, part](const index_t& s) {
	  return !(all_parallel_to({s}, part, n, 1.0) ||
		   all_orthogonal_to({s}, part, n, 1.0));
	});

    return not_vert_or_horiz;
  }

  std::vector<std::vector<index_t> >
  chamfer_regions(const triangular_mesh& mesh, const point n) {
    vector<index_t> non_prismatic_faces =
      non_prismatic_millable_faces(n, mesh);

    return normal_delta_regions(non_prismatic_faces, mesh, 180.0);
  }

}

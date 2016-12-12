#include "geometry/triangular_mesh.h"
#include "synthesis/toolpath.h"

namespace gca {

  toolpath
  freeform_finish_lines(const std::vector<index_t>& inds,
			const triangular_mesh& mesh,
			const tool& t,
			const double safe_z,
			const double stepover_fraction);

}

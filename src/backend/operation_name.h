#pragma once

namespace gca {

  enum pocket_name {
    FREEFORM_POCKET,
    FACE_POCKET,
    CONTOUR_POCKET,
    CONTOUR,
    FLAT_POCKET,
    TRACE_POCKET,
    CHAMFER_POCKET,
    DRILLED_HOLE_POCKET,
    SLICE_ROUGHING_POCKET
  };

  std::ostream& operator<<(std::ostream& out, const pocket_name p);

}

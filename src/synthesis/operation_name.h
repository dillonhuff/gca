#pragma once

namespace gca {

  enum pocket_name {
    FREEFORM_POCKET,
    FACE_POCKET,
    CONTOUR_POCKET,
    CONTOUR,
    FLAT_POCKET,
    TRACE_POCKET
  };

  std::ostream& operator<<(std::ostream& out, const pocket_name p);

}

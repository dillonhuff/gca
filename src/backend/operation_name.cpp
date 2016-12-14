#include <iostream>

#include "backend/operation_name.h"
#include "utils/check.h"

namespace gca {

  std::ostream& operator<<(std::ostream& out, const pocket_name p) {
    switch (p) {
    case FREEFORM_POCKET:
      out << "FREEFORM_POCKET";
      break;
    case FACE_POCKET:
      out << "FACE_POCKET";
      break;
    case CONTOUR:
      out << "CONTOUR";
      break;
    case CONTOUR_POCKET:
      out << "CONTOUR_POCKET";
      break;
    case FLAT_POCKET:
      out << "FLAT_POCKET";
      break;
    case TRACE_POCKET:
      out << "TRACE_POCKET";
      break;
    default:
      DBG_ASSERT(false);
    }
    return out;
  }

}

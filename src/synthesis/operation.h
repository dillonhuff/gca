#ifndef GCA_OPERATION_H
#define GCA_OPERATION_H

#include <iostream>

namespace gca {

  enum pocket_name {
    FREEFORM_POCKET,
    FACE_POCKET,
    CONTOUR_POCKET,
    FLAT_POCKET
  };

  std::ostream& operator<<(std::ostream& out, const pocket_name p);


}

#endif

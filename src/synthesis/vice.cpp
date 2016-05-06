#include "synthesis/vice.h"

namespace gca {

  vice emco_vice(const point loc) {
    return vice(loc, 5.0, 6.0, 0.75, 1.5, 0.5);
  }
}

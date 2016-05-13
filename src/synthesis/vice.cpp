#include "synthesis/vice.h"

namespace gca {

  vice emco_vice(const point loc) {
    return vice(loc, 2.5, 5.5, 1.1, 1.87, 1.3);
  }

  vice current_setup() {
    return emco_vice(point(-0.8, -4.4, -6.3));
  }
}

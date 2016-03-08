#ifndef GCA_SCALE_XYZ_H
#define GCA_SCALE_XYZ_H

#include "core/callback.h"

namespace gca {

  gprog* scale_xyz(double s, gprog& p);

  gprog* scale_xy(double sf, gprog& p);  
}

#endif

#ifndef GCA_OUTPUT_H
#define GCA_OUTPUT_H

#include <vector>

#include "context.h"
#include "gprog.h"

using namespace std;

namespace gca {

  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts);
    
}

#endif

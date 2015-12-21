#ifndef GCA_OUTPUT_H
#define GCA_OUTPUT_H

#include <vector>

#include "context.h"
#include "gprog.h"

using namespace std;

namespace gca {

  gprog* gcode_for_cuts(context& c, vector<cut*>& cuts);

  cut* sink_cut(context& c, cut* s, double l);

  void insert_sink_cuts(context& c, double l, vector<cut*>& cuts, vector<cut*>& dest);
    
}

#endif

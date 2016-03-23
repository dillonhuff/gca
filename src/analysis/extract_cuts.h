#ifndef GCA_EXTRACT_CUTS_H
#define GCA_EXTRACT_CUTS_H

#include "analysis/machine_state.h"
#include "core/gprog.h"
#include "core/lexer.h"

namespace gca {

  void extract_cuts(gprog* p, vector<vector<machine_state>>& ms);

  void extract_cuts(const vector<block> blocks, vector<vector<machine_state>>& ms);

}

#endif

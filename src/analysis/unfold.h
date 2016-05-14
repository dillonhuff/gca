#ifndef GCA_UNFOLD_H
#define GCA_UNFOLD_H

#include "gcode/lexer.h"

namespace gca {

  vector<block> unfold_gprog(const vector<block>& p);
}

#endif

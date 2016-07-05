#ifndef GCA_BLOCK_RATE_CHECKER_H
#define GCA_BLOCK_RATE_CHECKER_H

#include "gcode/lexer.h"
#include "gcode/cut.h"

namespace gca {

  bool all_cuts_within_block_rate(const vector<vector<cut*>>& blocks,
				  double blocks_per_second);
  
  bool all_cuts_within_block_rate(const vector<block>& blocks,
				  double blocks_per_second);
}

#endif

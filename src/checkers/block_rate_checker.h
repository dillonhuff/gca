#ifndef GCA_BLOCK_RATE_CHECKER_H
#define GCA_BLOCK_RATE_CHECKER_H

#include "core/lexer.h"

namespace gca {

  bool all_cuts_within_block_rate(const vector<block>& blocks,
				  double blocks_per_second);
}

#endif

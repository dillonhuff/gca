#ifndef GCA_CUT_TO_GCODE_H
#define GCA_CUT_TO_GCODE_H

#include "gcode/lexer.h"
#include "gcode/cut.h"
#include "backend/shape_layout.h"

namespace gca {

  vector<block> gcode_blocks_for_cuts(const vector<cut*>& cuts,
				      const cut_params& params);

  void append_cut_block(const cut* ci, vector<block>& blocks);

  void append_cuts_gcode_blocks(const vector<cut*>& cuts,
				vector<block>& blocks,
				const cut_params& params);

  std::vector<block> camaster_tool_change_block(const int tool_no);
}

#endif

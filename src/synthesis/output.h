#ifndef GCA_OUTPUT_H
#define GCA_OUTPUT_H

#include <vector>

#include "core/gprog.h"
#include "core/lexer.h"
#include "geometry/line.h"
#include "synthesis/linear_cut.h"
#include "synthesis/machine.h"

using namespace std;

namespace gca {

  void append_footer_blocks(vector<block>& blocks, machine_name m);
  void append_drag_knife_transfer(gprog* p, machine_name m);
  void append_drill_header_block(vector<block>& p, machine_name m);
  void append_drag_knife_transfer_block(vector<block>& p, machine_name m);

  vector<cut*> from_to_with_G0_height(point current_loc,
				      point next_loc,
				      double safe_height,
				      value* feedrate);
}

#endif

#include <sstream>

#include "analysis/extract_cuts.h"
#include "synthesis/cut.h"
#include "system/algorithm.h"

namespace gca {

  vector<cut*> extract_cuts(const vector<block>& states) {
    vector<cut*> cuts;
    return cuts;
  }

  void extract_cuts(const vector<block> blocks, vector<vector<machine_state>>& ms) {
    auto s = all_program_states(blocks);
    split_by(s, ms, [](const machine_state& c, const machine_state& p)
	     { return c.active_move_type == p.active_move_type;});
    delete_if(ms, [](const vector<machine_state>& c)
	      { return c.front().active_move_type == FAST_MOVE ||
		  c.front().active_move_type == UNKNOWN_MOVE_TYPE; });
  }

  void extract_cuts(gprog* p, vector<vector<machine_state>>& ms) {
    stringstream s;
    s << *p;
    auto ws = lex_gprog(s.str());
    extract_cuts(ws, ms);
  }
  
}

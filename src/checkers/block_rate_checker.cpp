#include "analysis/gcode_to_cuts.h"
#include "checkers/block_rate_checker.h"
#include "synthesis/cut.h"

namespace gca {

  bool all_cuts_within_block_rate(const vector<vector<cut*>>& paths,
				  double blocks_per_second) {
    assert(!within_eps(blocks_per_second, 0.0));
    double seconds_per_block = 1 / blocks_per_second;
    for (auto path : paths) {
      for (auto c : path) {
	if (cut_execution_time_seconds(c) <= seconds_per_block) {
	  cout << *c << endl;
	  cout << "has execution time = " << cut_execution_time_seconds(c) << endl;
	  return false;
	}
      }
    }
    return true;
  }

  bool all_cuts_within_block_rate(const vector<block>& blocks,
				  double blocks_per_second) {
    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(blocks, paths);
    if (r != GCODE_TO_CUTS_SUCCESS) {
      return false;
    }
    return all_cuts_within_block_rate(paths, blocks_per_second);
  }
}


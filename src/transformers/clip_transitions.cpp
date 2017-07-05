#include "transformers/clip_transitions.h"

#include "backend/output.h"
#include "utils/algorithm.h"

namespace gca {

  vector<cut*> clip_transition_heights(const vector<cut*>& path,
				       double new_safe_height) {
    vector<vector<cut*>> move_sequences =
      group_unary(path, [](const cut* c) { return c->is_safe_move(); });
    delete_if(move_sequences, [](const vector<cut*>& cs)
	      { return cs.front()->is_safe_move(); });
    bool all_normal_paths = true;
    for (auto s : move_sequences) {
      if (!is_vertical(s.front()) || s.size() < 2) {
	all_normal_paths = false;
	break;
      }
    }
    vector<cut*> clipped_cuts;
    if (all_normal_paths) {
      cout << "All normal paths" << endl;
      vector<vector<cut*>> transitions(move_sequences.size() - 1);
      // TODO: Add push feedrate inference
      auto mk_transition = [new_safe_height](const vector<cut*> l,
					     const vector<cut*> r) {
	return from_to_with_G0_height(l.back()->get_end(),
				      r.front()->get_start(),
				      new_safe_height, lit::make(10.0));
      };
      apply_between(move_sequences.begin() + 1, move_sequences.end(),
		    transitions.begin(),
		    mk_transition);
      for (unsigned i = 0; i < move_sequences.size(); i++) {
	auto m = move_sequences[i];
	clipped_cuts.insert(end(clipped_cuts), m.begin() + 1, m.end());
	if (i < move_sequences.size() - 1) {
	  auto t = transitions[i];
	  clipped_cuts.insert(end(clipped_cuts), t.begin(), t.end());
	}
      }
    } else {
      clipped_cuts = path;
    }
    return clipped_cuts;
  }

  vector<vector<cut*>> clip_transition_heights(vector<vector<cut*>>& paths,
					       double new_safe_height) {
    vector<vector<cut*>> clipped_paths;
    for (auto path : paths) {
      clipped_paths.push_back(clip_transition_heights(path, new_safe_height));
    }
    return clipped_paths;
  }


}

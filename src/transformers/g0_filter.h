#ifndef GCA_G0_FILTER_H
#define GCA_G0_FILTER_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/pass.h"

namespace gca {

  class g0_filter_state : public per_instr_state {
    unsigned int skip_irrelevant_G0_instrs(unsigned int i,
					   vector<point>& positions,
					   gprog* p) {
      unsigned int n = i + 1;
      unsigned int last_net_zero_pos = i;
      while (n < p->size()) {
	instr ist = *((*p)[n]);
	if (!ist.is_G0()) { break; }
	if (within_eps(positions[n], positions[i])) {
	  last_net_zero_pos = n;
	} 
	n++;
      }
      return last_net_zero_pos;
    }
    
  public:
    gprog* p;
    vector<point> positions;
    int j;

  g0_filter_state(pass& tp) : per_instr_state(tp) {
      p = mk_gprog();
      j = 0;
    }

    // TODO: Adjust this algorithm to avoid using
    // all_positions_starting_at.
    // TODO: Find a more sensible way to structure
    // code that needs to skip instructions in the iteration
    void update_G() {
      int curr = t.i;
      if (curr != j) { return; }
      gprog* n = t.p;
      vector<point> positions;
      n->all_positions_starting_at(point(0, 0, 0), positions);
      positions.erase(positions.begin());
      instr* i = (*n)[j];
      if (i->is_G()) {
	unsigned int next_pos = skip_irrelevant_G0_instrs(j, positions, n);
	p->push_back(i);
	if (next_pos == j) {
	  j++;
	} else {
	  j = next_pos + 1;
	}
      } else {
	n->push_back(i);
	j++;
      }
    }

    void update_G0(move_instr& ist) { update_G(); }
    void update_G1(move_instr& ist) { update_G(); }
    void update_default(instr& ist) { p->push_back(&ist); }
  };

  gprog* filter_G0_moves(gprog* p) {
    pass ps;
    g0_filter_state filter_s(ps);
    ps.add_state(GCA_G0_FILTER_STATE, &filter_s);
    ps.exec(p);
    return ps.get_state<g0_filter_state>(GCA_G0_FILTER_STATE)->p;
  }

}

#endif

#ifndef GCA_G0_FILTER_H
#define GCA_G0_FILTER_H

#include "context.h"

namespace gca {

  class g0_filter {
  protected:
    unsigned int skip_irrelevant_G0_instrs(unsigned int i,
					   vector<point>& positions,
					   gprog* p) {
      unsigned int n = i + 1;
      unsigned int last_net_zero_pos = i;
      while (n < p->size()) {
	instr ist = *((*p)[n]);
	bool is_g0 = ist.is_G() && ist.v == 0;
	if (!is_g0) { break; }
	if (within_eps(positions[n], positions[i])) {
	  last_net_zero_pos = n;
	} 
	n++;
      }
      return last_net_zero_pos;
    }
    
  public:
    virtual gprog* apply(context& c, gprog* p) {
      vector<point> positions;
      p->all_positions_starting_at(point(0, 0, 0), positions);
      // TODO: Adjust this algorithm to use all_positions_starting_at
      // correctly rather than just eliminating the starting position
      positions.erase(positions.begin());
      gprog* n = c.mk_gprog();
      for (unsigned int j = 0; j < p->size();) {
	instr* i = (*p)[j];
	if (i->is_G()) {
	  unsigned int next_pos = skip_irrelevant_G0_instrs(j, positions, p);
	  n->push_back(i);
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
      return n;
    }
  };

}

#endif


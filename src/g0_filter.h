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
	instr ist = (*p)[n];
	if (ist.is_G());
	if (within_eps(positions[n], positions[i]) && ist.is_G() && is.v == 0) {
	  break;
	}
      }
      return last_net_zero_pos;
    }
    
  public:
    virtual gprog* apply(context& c, gprog* p) {
      vector<point> positions = p->all_positions();
      gprog* n = c.mk_gprog();
      for (unsigned int j = 0; j < p->size();) {
	instr* i = (*p)[j];
	if (i->is_G() && i->v == 0) {
	  unsigned int next_pos = skip_irrelevant_G0_instrs(j, positions, p);
	  if (next_pos == j) {
	    n->push_back(i);
	    j++;	    
	  } else {
	    j = next_pos;
	    
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


#include <algorithm>

#include "synthesis/schedule_cuts.h"

namespace gca {

  bool cmp_z(const cut* l, const cut* r) {
    assert(within_eps(l->start.z, l->end.z));
    assert(within_eps(r->start.z, r->end.z));
    return l->start.z > r->start.z;
  }

  struct elem_test {
    const vector<cut*>& elems;
    elem_test(const vector<cut*>& pelems) : elems(pelems) {}
    bool operator()(const cut* t) {
      return find(elems.begin(), elems.end(), t) != elems.end();
    }
  };

  bool are_contiguous(const cut* last, const cut* next) {
    if (last->is_linear_cut() && next->is_linear_cut()) {
      return within_eps(last->end, next->start);
    } else {
      assert(false);
    }
  }

  vector<cut*> contiguous_chain(vector<cut*>::const_iterator it,
				vector<cut*>::const_iterator e) {
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    vector<cut*> cont_chain;
    for (; it != e; ++it) {
      next_cut = *it;
      if (last_cut == NULL || are_contiguous(last_cut, next_cut)) {
	cont_chain.push_back(next_cut);
	last_cut = next_cut;
      }
    }
    return cont_chain;
  }

  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    vector<cut*> scheduled_cuts = cuts;
    vector<cut*>::iterator it = scheduled_cuts.begin();
    while (it < scheduled_cuts.end()) {
      vector<cut*> next_chain = contiguous_chain(it, scheduled_cuts.end());
      elem_test rt(next_chain);
      stable_partition(it, scheduled_cuts.end(), rt);
      it += next_chain.size();
    }
    return scheduled_cuts;
  }

}

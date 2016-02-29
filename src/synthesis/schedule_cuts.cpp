#include "synthesis/schedule_cuts.h"

namespace gca {

  bool cmp_z(const cut* l, const cut* r) {
    assert(within_eps(l->start.z, l->end.z));
    assert(within_eps(r->start.z, r->end.z));
    return l->start.z > r->start.z;
  }

  struct remove_test {
    const vector<cut*>& to_remove;
    remove_test(const vector<cut*>& pto_remove) : to_remove(pto_remove) {}
    bool operator()(const cut* t) {
      return find(to_remove.begin(), to_remove.end(), t) != to_remove.end();
    }
  };

  bool are_contiguous(const cut* last, const cut* next) {
    if (last->is_linear_cut() && next->is_linear_cut()) {
      return within_eps(last->end, next->start);
    } else {
      assert(false);
    }
  }

  vector<cut*> contiguous_chain(const vector<cut*>& cuts) {
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    vector<cut*> cont_chain;
    for (vector<cut*>::const_iterator it = cuts.begin(); it != cuts.end(); ++it) {
      next_cut = *it;
      if (last_cut == NULL || are_contiguous(last_cut, next_cut)) {
	cont_chain.push_back(next_cut);
	last_cut = next_cut;
      }
    }
    return cont_chain;
  }

  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    vector<cut*> scuts = cuts;
    vector<cut*> scheduled_cuts;
    while (scuts.size() > 0) {
      vector<cut*> next_chain = contiguous_chain(scuts);
      scheduled_cuts.insert(scheduled_cuts.end(),
      			    next_chain.begin(), next_chain.end());
      remove_test rt(next_chain);      
      scuts.erase(remove_if(scuts.begin(), scuts.end(), rt), scuts.end());
    }
    //stable_sort(scheduled_cuts.begin(), scheduled_cuts.end(), cmp_z);
    return scheduled_cuts;
  }

}

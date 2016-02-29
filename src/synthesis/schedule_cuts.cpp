#include "synthesis/schedule_cuts.h"

namespace gca {

  bool cmp_z(const cut* l, const cut* r) {
    assert(within_eps(l->start.z, l->end.z));
    assert(within_eps(r->start.z, r->end.z));
    return l->start.z > r->start.z;
  }

  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    vector<cut*> scheduled_cuts;
    for (vector<cut*>::const_iterator it = cuts.begin(); it != cuts.end(); ++it) {
      scheduled_cuts.push_back(*it);
    }
    stable_sort(scheduled_cuts.begin(), scheduled_cuts.end(), cmp_z);
    return scheduled_cuts;
  }

}

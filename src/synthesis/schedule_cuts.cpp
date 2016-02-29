#include "synthesis/schedule_cuts.h"

namespace gca {

  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    vector<cut*> scheduled_cuts;
    for (vector<cut*>::const_iterator it = cuts.begin(); it != cuts.end(); ++it) {
      scheduled_cuts.push_back(*it);
    }
    return scheduled_cuts;
  }

}

#ifndef GCA_FEED_CHANGER_H
#define GCA_FEED_CHANGER_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/pass.h"

#define GCA_FEED_CHANGER_STATE 2001

namespace gca {
  gprog* change_feeds(gprog* p, value* initial_feedrate, value* new_feedrate);
  gprog* generalize_feeds(gprog* p, value* default_val, value* initial_feedratep, var* new_feedratep);

}
#endif

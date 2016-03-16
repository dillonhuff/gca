#ifndef GCA_FEED_CHANGER_H
#define GCA_FEED_CHANGER_H

#include "core/lexer.h"

namespace gca {

  vector<block> change_feeds(const vector<block>& p, value*
			     initial_feedrate, value* new_feedrate);  
}
#endif

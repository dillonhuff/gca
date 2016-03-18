#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "synthesis/cut.h"

using namespace std;

namespace gca {
  
  typedef vector<cut*> cut_group;
  
  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change);
  
}

#endif

#ifndef GCA_PROFILER_H
#define GCA_PROFILER_H

#include <vector>

#include "synthesis/cut.h"

using namespace std;

namespace gca {

  void print_profile_info(vector<cut*>& path);

}

#endif

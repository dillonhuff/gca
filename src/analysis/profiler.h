#ifndef GCA_PROFILER_H
#define GCA_PROFILER_H

#include <vector>

#include "gcode/cut.h"

using namespace std;

namespace gca {

  struct profile_info {
    double time,
      time_wo_transitions,
      time_wo_G1s,
      time_wo_G2_G3 = 0.0,
      inches_traveled = 0.0;
  };

  struct program_profile_info {
    vector<profile_info> toolpath_profiles;

    inline size_t size() const { return toolpath_profiles.size(); }

    vector<profile_info>::iterator begin() { return toolpath_profiles.begin(); }
    vector<profile_info>::iterator end() { return toolpath_profiles.end(); }
    
    vector<profile_info>::const_iterator begin() const
    { return toolpath_profiles.begin(); }
    vector<profile_info>::const_iterator end() const
    { return toolpath_profiles.end(); }

    void push_back(profile_info p)
    { toolpath_profiles.push_back(p); }

  };

  double execution_time(const program_profile_info& p);

  program_profile_info profile_toolpaths(const vector<vector<cut*>>& paths);
  void print_profile_info(const vector<cut*>& path);

}

#endif

#include "analysis/profiler.h"

namespace gca {

  profile_info path_profile_info(const vector<cut*>& path) {
    profile_info info;
    info.time = execution_time_minutes(path);
    info.time_wo_transitions = 0.0;
    info.time_wo_G1s = 0.0;
    info.time_wo_G2_G3 = 0.0;
    info.inches_traveled = 0.0;

    for (auto c : path) {
      if (!c->is_safe_move())
	{ info.time_wo_transitions += cut_execution_time_minutes(c); }
      if (!c->is_linear_cut())
	{ info.time_wo_G1s += cut_execution_time_minutes(c); }
      if (!c->is_circular_arc() && !c->is_circular_helix_cut())
	{ info.time_wo_G2_G3 += cut_execution_time_minutes(c); }
      info.inches_traveled += (c->get_end() - c->get_start()).len();
    }
    return info;
  }

  void print_profile_info(const profile_info& info) {
    double pct_time_in_G0s = ((info.time - info.time_wo_transitions) / info.time) * 100;
    double pct_time_in_G1s = ((info.time - info.time_wo_G1s) / info.time) * 100;
    double pct_time_in_G2s_G3s = ((info.time - info.time_wo_G2_G3) / info.time) * 100;
    double total_pct = pct_time_in_G0s + pct_time_in_G1s + pct_time_in_G2s_G3s;
    cout << "PATH STATISTICS" << endl;
    cout << "---------------------------------------------------------" << endl;
    cout << "total inches traveled           = " << info.inches_traveled << endl;
    cout << "execution time                  = " << info.time << " minutes" << endl;
    cout << "% of time spent in G0 moves     = " << pct_time_in_G0s << endl;
    cout << "% of time spent in G1 moves     = " << pct_time_in_G1s << endl;
    cout << "% of time spent in G2, G3 moves = " << pct_time_in_G2s_G3s << endl;
    cout << "Total pct                       = " << total_pct << endl;
    assert(within_eps(total_pct, 100.0));
  }

  void print_profile_info(const vector<cut*>& path) {
    auto info = path_profile_info(path);
    print_profile_info(info);
  }

  program_profile_info profile_toolpaths(const vector<vector<cut*>>& paths) {
    program_profile_info info;
    for (auto p : paths)
      { info.push_back(path_profile_info(p)); }
    return info;
  }

  double execution_time(const program_profile_info& p) {
    double time = 0;
    for (auto prof : p)
      { time += prof.time; }
    return time;
  }
}

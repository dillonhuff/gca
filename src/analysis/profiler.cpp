#include "analysis/profiler.h"

namespace gca {

  void print_profile_info(vector<cut*>& path) {
    double time = execution_time_minutes(path);
    double time_wo_transitions = 0.0;
    double time_wo_G1s = 0.0;
    double time_wo_G2_G3 = 0.0;
    double inches_traveled = 0.0;

    for (auto c : path) {
      if (!c->is_safe_move())
	{ time_wo_transitions += cut_execution_time_minutes(c); }
      if (!c->is_linear_cut())
	{ time_wo_G1s += cut_execution_time_minutes(c); }
      if (!c->is_circular_arc() && !c->is_circular_helix_cut())
	{ time_wo_G2_G3 += cut_execution_time_minutes(c); }
      inches_traveled += (c->get_end() - c->get_start()).len();
    }
    double pct_time_in_G0s = ((time - time_wo_transitions) / time) * 100;
    double pct_time_in_G1s = ((time - time_wo_G1s) / time) * 100;
    double pct_time_in_G2s_G3s = ((time - time_wo_G2_G3) / time) * 100;
    double total_pct = pct_time_in_G0s + pct_time_in_G1s + pct_time_in_G2s_G3s;
    cout << "PATH STATISTICS" << endl;
    cout << "---------------------------------------------------------" << endl;
    cout << "total inches traveled           = " << inches_traveled << endl;
    cout << "execution time                  = " << time << " minutes" << endl;
    cout << "% of time spent in G0 moves     = " << pct_time_in_G0s << endl;
    cout << "% of time spent in G1 moves     = " << pct_time_in_G1s << endl;
    cout << "% of time spent in G2, G3 moves = " << pct_time_in_G2s_G3s << endl;
    cout << "Total pct                       = " << total_pct << endl;
    assert(within_eps(total_pct, 100.0));
  }

}

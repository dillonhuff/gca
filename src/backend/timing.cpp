#include "backend/timing.h"

namespace gca {

  void increment(fab_plan_timing_info& total_time,
		 const fab_plan_timing_info& inc_time) {
    total_time.toolpath_time_seconds += inc_time.toolpath_time_seconds;
    total_time.air_time_seconds += inc_time.air_time_seconds;
  }

  fab_plan_timing_info make_timing_info(const toolpath& tp,
					const double rapid_feed) {
    double exec_time = execution_time_seconds(tp, rapid_feed);
    double air_time = air_time_seconds(tp, rapid_feed);
    double air_pct = (air_time / exec_time) * 100.0;

    cout << "Toolpath execution time = " << exec_time << " seconds " << endl;
    cout << "Toolpath air time = " << air_time << " seconds " << endl;
    cout << "Fraction of toolpath in air = " << air_pct << " % " << endl;

    return fab_plan_timing_info(exec_time, air_time);
  }

  fab_plan_timing_info make_timing_info(const fabrication_setup& step,
					const double rapid_feed) {
    fab_plan_timing_info step_time;

    for (auto& tp : step.toolpaths()) {
      auto toolpath_time = make_timing_info(tp, rapid_feed);
      increment(step_time, toolpath_time);
    }

    return step_time;
  }

  void print_time_info(std::ostream& out,
		       const fab_plan_timing_info& times) {
    out << "Total execution time so far = " << times.toolpath_time_seconds << " seconds" << endl;
    out << "Total air time so far = " << times.air_time_seconds << " seconds" << endl;

    double total_air_pct =
      (times.air_time_seconds / times.toolpath_time_seconds) * 100.0;

    out << "Total fraction of air time = " << total_air_pct << " % " << endl;
  }

}

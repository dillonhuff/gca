#pragma once

#include "synthesis/fabrication_plan.h"
#include "backend/toolpath.h"

namespace gca {

  struct fab_plan_timing_info {
    double toolpath_time_seconds;
    double air_time_seconds;

    fab_plan_timing_info() : toolpath_time_seconds(0.0), air_time_seconds(0.0) {}

    fab_plan_timing_info(double tts, double ats) :
      toolpath_time_seconds(tts),
      air_time_seconds(ats) {}
  };

  void increment(fab_plan_timing_info& total_time,
		 const fab_plan_timing_info& inc_time);

  fab_plan_timing_info make_timing_info(const toolpath& tp,
					const double rapid_feed);

  fab_plan_timing_info make_timing_info(const fabrication_setup& step,
					const double rapid_feed);

  void print_time_info(std::ostream& out,
		       const fab_plan_timing_info& times);
  
}

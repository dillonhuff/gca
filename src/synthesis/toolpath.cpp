#include "synthesis/toolpath.h"

namespace gca {

  double cut_time_seconds(const toolpath& tp) {
    double cut_distance = 0.0;

    for (auto& pl : tp.lines) {
      cut_distance += length(pl);
    }

    return (cut_distance / tp.feedrate)*60.0;
  }

  // NOTE: Assumes Square transitions
  double air_time_seconds(const toolpath& tp,
			  const double rapid_feed) {
    if (tp.lines.size() < 2) { return 0.0; }

    double horizontal_air_distance_inches = 0.0;
    double up_air_distance_inches = 0.0;
    double down_air_distance_inches = 0.0;

    for (unsigned i = 0; i < tp.lines.size() - 1; i++) {
      const point& last = tp.lines[i].back();
      const point& next = tp.lines[i + 1].front();

      double up_distance = tp.safe_z_before_tlc - last.z;
      double down_distance = tp.safe_z_before_tlc - next.z;
      point horiz_diff = next - last;
      horiz_diff.z = 0.0;
      double horizontal_distance = horiz_diff.len();

      horizontal_air_distance_inches += horizontal_distance;
      up_air_distance_inches += up_distance;
      down_air_distance_inches += down_distance;
    }

    double horizontal_time = (horizontal_air_distance_inches / rapid_feed)*60;
    double up_time = (up_air_distance_inches / rapid_feed)*60;
    double down_time = (down_air_distance_inches / tp.plunge_feedrate)*60;
    
    return horizontal_time + up_time + down_time;
  }
  
  double execution_time_seconds(const toolpath& tp,
				const double rapid_feed) {
    double exec_time = cut_time_seconds(tp);
    exec_time += air_time_seconds(tp, rapid_feed);
    return exec_time;
  }

}

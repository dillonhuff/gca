#include "synthesis/gcode_generation.h"
#include "synthesis/toolpath.h"

namespace gca {

  toolpath::toolpath(const pocket_name& p_pocket_type,
		     const double p_safe_z,
		     const double p_spindle,
		     const double p_feed,
		     const double p_plunge_feed,
		     const tool& p_t,
		     const std::vector<polyline>& p_lines)
      : pocket_tp(p_pocket_type),
	safe_z_before_tlc(p_safe_z),
	spindle_speed(p_spindle),
	feedrate(p_feed),
	plunge_feedrate(p_plunge_feed),
	t(p_t),
	ls(p_lines) {

    // TODO: Deal with machine specification
    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = safe_z_before_tlc;
    params.set_plunge_feed(plunge_feedrate);
    
    cuts = polylines_to_cuts(lines(), tool_number(), params, spindle_speed, feedrate);
  }
  
  double cut_time_seconds(const toolpath& tp) {
    double cut_distance = 0.0;

    for (auto& pl : tp.lines()) {
      cut_distance += length(pl);
    }

    return (cut_distance / tp.feedrate)*60.0;
  }

  // NOTE: Assumes Square transitions
  double air_time_seconds(const toolpath& tp,
			  const double rapid_feed) {
    
    if (tp.lines().size() < 2) { return 0.0; }

    double horizontal_air_distance_inches = 0.0;
    double up_air_distance_inches = 0.0;
    double down_air_distance_inches = 0.0;

    for (unsigned i = 0; i < tp.lines().size() - 1; i++) {
      const point& last = tp.lines()[i].back();
      const point& next = tp.lines()[i + 1].front();

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

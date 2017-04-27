#pragma once

#include "backend/cut_params.h"
#include "gcode/cut.h"
#include "gcode/lexer.h"
#include "simulators/region.h"

#include <map>

#include <boost/optional.hpp>

namespace gca {

  enum tool_end {
    ROUGH_ENDMILL,
    FINISH_ENDMILL,
    BALL_ENDMILL,
    DRILL_ENDMILL,
    FACE_ENDMILL,
    SPOT_DRILL_ENDMILL,
    COUNTERSINK_ENDMILL,
    KEY_CUTTER_ENDMILL,
    FLY_CUTTER_ENDMILL
  };


  struct tool_info {
    tool_end tool_end_type;
    double tool_diameter;
  };

  // TODO: Templatize the extra metadata?
  struct operation_range {
    std::string name;
    int start_line, end_line;
    int tool_number;
  };

  struct operation_params {

    int current_tool_no;
    tool_end tool_end_type;
    double tool_diameter;

    double cut_depth;
    double feedrate;
    double spindle_speed;
    double sfm;

    double total_distance;
    double cut_distance;

    double total_time;
    double cut_time;

    double material_removed;

    std::string file_name;

    operation_range range;

    double SFM() const {
      return surface_feet_per_minute(spindle_speed, tool_diameter);
    }

    double average_MRR() const {
      return material_removed / cut_time;
    }

  };

  enum operation_type {
    ROUGH_OPERATION,
    FINISH_OPERATION,
    OTHER_OPERATION
  };

  struct cut_simulation_log {
    cut* c;
    std::vector<point_update> updates;
  };

  struct operation_info {
    operation_range range;
    tool_info tool_inf;
  };

  struct operation_log {
    std::vector<cut_simulation_log> cuts;
    operation_info info;
  };

  struct simulation_log {
    double resolution;
    std::vector<operation_log> operation_logs;
  };
  
  std::map<int, tool_info> infer_tool_table_GCA(const std::vector<block>& p);
  std::map<int, tool_info> infer_tool_table_HAAS(const std::vector<block>& p);

  boost::optional<double> infer_program_length_feet(const std::vector<block>& p);

  std::vector<operation_range>
  infer_operation_ranges_GCA(const std::vector<block>& p);

  std::vector<operation_range>
  infer_operation_ranges_HAAS(const std::vector<block>& p);

  std::ostream& operator<<(std::ostream& out, const operation_range& op_range);
  std::ostream& operator<<(std::ostream& out, const operation_params& op);

  std::vector<operation_params>
  program_operations_GCA(std::vector<std::vector<cut*> >& paths,
			 map<int, tool_info>& tool_table,
			 const std::vector<operation_range>& op_ranges);

  std::vector<operation_params>
  program_operations_HAAS(std::vector<std::vector<cut*> >& paths,
			  map<int, tool_info>& tool_table,
			  const std::vector<operation_range>& op_ranges);
  
  double estimate_feedrate_median(const std::vector<cut*>& path);

  double estimate_spindle_speed_median(const std::vector<cut*>& path);

  double estimate_cut_depth_median(const std::vector<cut*>& path);

  double estimate_cut_depth_mean(const std::vector<cut*>& path);

  simulation_log
  simulation_log_HAAS(std::vector<std::vector<cut*> >& paths,
		      map<int, tool_info>& tool_table,
		      const std::vector<operation_range>& op_ranges);
  
}

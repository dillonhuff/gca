#pragma once

#include "backend/cut_params.h"
#include "gcode/lexer.h"


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

  
  struct operation_range {
    std::string name;
    int start_line, end_line;
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

  
}

#ifndef GCA_MACHINE_STATE_H
#define GCA_MACHINE_STATE_H

#include "core/lexer.h"

namespace gca {

  enum canned_cycle_return_policy {
    CANNED_CYCLE_RETURN_POLICY_UNKNOWN = 0,
    CANNED_CYCLE_RETURN_Z,
    CANNED_CYCLE_RETURN_R
  };

  enum coolant_state {
    COOLANT_STATE_UNKNOWN = 0,
    COOLANT_MIST,
    COOLANT_FLOOD,
    COOLANT_OFF
  };

  enum spindle_state {
    SPINDLE_STATE_UNKNOWN = 0,
    SPINDLE_CLOCKWISE,
    SPINDLE_COUNTERCLOCKWISE,
    SPINDLE_OFF
  };

  enum non_modal_setting {
    NO_NON_MODAL_SETTING = 0,
    MOVE_HOME_THROUGH_POINT,
    POSITION_REGISTER_MOVE
  };

  enum tool_plane {
    UNKNOWN_PLANE = 0,
    XY_PLANE,
    YZ_PLANE,
    ZX_PLANE
  };

  enum tool_height_compensation {
    TOOL_HEIGHT_COMP_UNKNOWN = 0,
    TOOL_HEIGHT_COMP_OFF,
    TOOL_HEIGHT_COMP_NEGATIVE,
    TOOL_HEIGHT_COMP_POSITIVE
  };

  enum tool_radius_compensation {
    TOOL_RADIUS_COMP_UNKNOWN = 0,
    TOOL_RADIUS_COMP_OFF,
    TOOL_RADIUS_COMP_LEFT,
    TOOL_RADIUS_COMP_RIGHT
  };
  
  enum coord_system {
    UNKNOWN_COORD_SYSTEM = 0,
    G54_COORD_SYSTEM,
    MACHINE_COORD_SYSTEM
  };

  enum distance_mode {
    UNKNOWN_DISTANCE_MODE = 0,
    ABSOLUTE_DISTANCE_MODE,
    RELATIVE_DISTANCE_MODE
  };

  enum move_type {
    UNKNOWN_MOVE_TYPE = 0,
    FAST_MOVE,
    LINEAR_MOVE,
    CLOCKWISE_CIRCULAR_MOVE,
    COUNTERCLOCKWISE_CIRCULAR_MOVE,
    CANCEL_CANNED_CYCLE_MOVE,
    CANNED_CYCLE_73_MOVE,
    CANNED_CYCLE_81_MOVE,
    CANNED_CYCLE_83_MOVE,
    CANNED_CYCLE_84_MOVE,
    CANNED_CYCLE_85_MOVE
  };

  struct machine_state {
    value* feedrate;
    value* spindle_speed;
    move_type active_move_type;
    distance_mode active_distance_mode;
    coord_system active_coord_system;
    tool_height_compensation tool_height_comp;
    tool_radius_compensation tool_radius_comp;
    tool_plane active_plane;
    non_modal_setting active_non_modal_setting;
    value* x;
    value* y;
    value* z;
    spindle_state spindle_setting;
    value* last_referenced_tool;
    value* active_tool;
    coolant_state coolant_setting;
    value* tool_height_value;
    value* i;
    value* j;
    value* k;
    value* tool_radius_value;
    canned_cycle_return_policy active_canned_cycle_return_policy;
    value* r;
    value* q;
    int line_no;

    machine_state() :
      feedrate(omitted::make()), spindle_speed(omitted::make()),
      active_move_type(UNKNOWN_MOVE_TYPE),
      active_distance_mode(UNKNOWN_DISTANCE_MODE),
      active_coord_system(UNKNOWN_COORD_SYSTEM),
      tool_height_comp(TOOL_HEIGHT_COMP_UNKNOWN),
      tool_radius_comp(TOOL_RADIUS_COMP_UNKNOWN),
      active_plane(UNKNOWN_PLANE),
      active_non_modal_setting(NO_NON_MODAL_SETTING),
      x(omitted::make()), y(omitted::make()), z(omitted::make()),
      spindle_setting(SPINDLE_STATE_UNKNOWN),
      last_referenced_tool(omitted::make()),
      active_tool(omitted::make()),
      coolant_setting(COOLANT_STATE_UNKNOWN),
      tool_height_value(omitted::make()),
      i(omitted::make()), j(omitted::make()), k(omitted::make()),
      tool_radius_value(omitted::make()),
      active_canned_cycle_return_policy(CANNED_CYCLE_RETURN_POLICY_UNKNOWN),
      r(omitted::make()), q(omitted::make()),
      line_no(-1) {}
  };

  machine_state next_machine_state(const block& b, const machine_state& s);
  vector<machine_state> all_program_states(const vector<block>& p);
  vector<machine_state> all_program_states(const machine_state& init,
					   const vector<block>& p);

  bool operator==(const machine_state& l, const machine_state& r);
  bool operator!=(const machine_state& l, const machine_state& r);
  ostream& operator<<(ostream& stream, const machine_state& s);
  ostream& operator<<(ostream& stream, const vector<machine_state>& s);
  ostream& operator<<(ostream& stream, const spindle_state s);
  ostream& operator<<(ostream& out, const move_type s);
  ostream& operator<<(ostream& out, const coord_system s);
  
}

#endif

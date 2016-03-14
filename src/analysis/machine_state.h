#ifndef GCA_MACHINE_STATE_H
#define GCA_MACHINE_STATE_H

#include "core/lexer.h"

namespace gca {

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
    COUNTERCLOCKWISE_CIRCULAR_MOVE
  };

  struct machine_state {
    value* feedrate;
    value* spindle_speed;
    move_type active_move_type;
    distance_mode active_distance_mode;

    machine_state() :
      feedrate(omitted::make()), spindle_speed(omitted::make()),
      active_move_type(UNKNOWN_MOVE_TYPE),
      active_distance_mode(UNKNOWN_DISTANCE_MODE) {}
    machine_state(const machine_state& s) :
      feedrate(s.feedrate), spindle_speed(s.spindle_speed),
      active_move_type(UNKNOWN_MOVE_TYPE),
      active_distance_mode(UNKNOWN_DISTANCE_MODE) {}

    machine_state& operator=(const machine_state& s) {
      feedrate = s.feedrate;
      spindle_speed = s.spindle_speed;
      active_move_type = s.active_move_type;
      active_distance_mode = s.active_distance_mode;
      return *this;
    }
  };

  machine_state next_machine_state(const block& b, const machine_state& s);

  bool operator==(const machine_state& l, const machine_state& r);
  bool operator!=(const machine_state& l, const machine_state& r);
  ostream& operator<<(ostream& stream, const machine_state& s);
}

#endif

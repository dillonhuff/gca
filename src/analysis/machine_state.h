#ifndef GCA_MACHINE_STATE_H
#define GCA_MACHINE_STATE_H

#include "core/lexer.h"

namespace gca {

  struct machine_state {
    value* feedrate;

    machine_state() : feedrate(omitted::make()) {}
    machine_state(const machine_state& s) : feedrate(s.feedrate) {}

    machine_state& operator=(const machine_state& s) {
      feedrate = s.feedrate;
      return *this;
    }
  };

  machine_state next_machine_state(const block& b, const machine_state& s);

  bool operator==(const machine_state& l, const machine_state& r);
  bool operator!=(const machine_state& l, const machine_state& r);
  ostream& operator<<(ostream& stream, const machine_state& s);
}

#endif

#include "analysis/machine_state.h"
#include "core/value.h"

namespace gca {

  struct matches_char {
    char c;
    matches_char(char cp) : c(cp) {}
    bool operator()(const token* t) {
      if (t->tp() == ICODE) {
	const icode* ic = static_cast<const icode*>(t);
	return ic->c == c;
      }
      return false;
    }
  };

  icode* find_icode(char c, const block& b) {
    block::const_iterator i = find_if(b.begin(), b.end(), matches_char(c));
    if (i == b.end()) { return NULL; }
    token* t = *i;
    return static_cast<icode*>(t);
  }

  machine_state next_machine_state(const block& b, const machine_state& s) {
    machine_state r = s;
    icode* f = find_icode('F', b);
    if (f) {
      assert(f->v.is_lit());
      const lit& fr = static_cast<const lit&>(f->v);
      r.feedrate = lit::make(fr.v);
    }
    icode* k = find_icode('S', b);
    if (k) {
      assert(k->v.is_lit());
      const lit& sr = static_cast<const lit&>(k->v);
      r.spindle_speed = lit::make(sr.v);
    }
    icode* g = find_icode('G', b);
    if (g) {
      assert(g->v.is_ilit());
      const ilit& sr = static_cast<const ilit&>(g->v);
      switch(sr.v) {
      case 0:
	r.active_move_type = FAST_MOVE;
	break;
      case 1:
	r.active_move_type = LINEAR_MOVE;
	break;
      case 2:
	r.active_move_type = CLOCKWISE_CIRCULAR_MOVE;
	break;
      case 3:
	r.active_move_type = COUNTERCLOCKWISE_CIRCULAR_MOVE;
	break;
      case 90:
	r.active_distance_mode = ABSOLUTE_DISTANCE_MODE;
	break;
      case 91:
	r.active_distance_mode = RELATIVE_DISTANCE_MODE;
	break;
      default:
	cout << "Unsupported g instruction: " << *g << endl;
	assert(false);
      }
    }
    return r;
  }

  bool operator==(const machine_state& l, const machine_state& r) {
    return *(l.feedrate) == *(r.feedrate) &&
      *(l.spindle_speed) == *(r.spindle_speed) &&
      l.active_move_type == r.active_move_type;
  }

  bool operator!=(const machine_state& l, const machine_state& r)
  { return !(l == r); }

  ostream& operator<<(ostream& stream, const machine_state& s) {
    stream << "F ";
    if (s.feedrate->is_omitted()) {
      stream << "<omitted>";
    } else {
      stream << *(s.feedrate);
    }
    stream << endl;
    return stream;
  }
  
}

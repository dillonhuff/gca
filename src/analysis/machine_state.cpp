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

  bool no_state_effect(const token* t) {
    if (t->tp() == ICODE) {
      const icode* ic = static_cast<const icode*>(t);
      return ic->c == 'N' || ic->c == 'O';
    }
    return true;
  }

  machine_state next_machine_state(const block& bs, const machine_state& s) {
    block b = bs;
    b.erase(remove_if(b.begin(), b.end(), no_state_effect), b.end());
    machine_state r = s;
    icode* f = find_icode('F', b);
    if (f) {
      assert(f->v.is_lit());
      const lit& fr = static_cast<const lit&>(f->v);
      r.feedrate = lit::make(fr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(f)), b.end());
    }
    icode* k = find_icode('S', b);
    if (k) {
      assert(k->v.is_lit());
      const lit& sr = static_cast<const lit&>(k->v);
      r.spindle_speed = lit::make(sr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(k)), b.end());
    }
    icode* x = find_icode('X', b);
    if (x) {
      assert(x->v.is_lit());
      const lit& sr = static_cast<const lit&>(x->v);
      r.spindle_speed = lit::make(sr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(x)), b.end());
    }
    icode* y = find_icode('Y', b);
    if (y) {
      assert(y->v.is_lit());
      const lit& sr = static_cast<const lit&>(y->v);
      r.spindle_speed = lit::make(sr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(y)), b.end());
    }
    icode* z = find_icode('Z', b);
    if (z) {
      assert(z->v.is_lit());
      const lit& sr = static_cast<const lit&>(z->v);
      r.spindle_speed = lit::make(sr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(z)), b.end());
    }

    icode* t = find_icode('T', b);
    if (t) {
      assert(t->v.is_ilit());
      const ilit& sr = static_cast<const ilit&>(t->v);
      r.last_referenced_tool = ilit::make(sr.v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(t)), b.end());
    }
    
    icode* m = find_icode('M', b);
    while (m != NULL) {
      assert(m->v.is_ilit());
      const ilit& sr = static_cast<const ilit&>(m->v);
      switch (sr.v) {
      case 3:
	r.spindle_setting = SPINDLE_CLOCKWISE;
	break;
      case 5:
	r.spindle_setting = SPINDLE_OFF;
	break;
      case 6:
	assert(r.last_referenced_tool->is_ilit());
	r.active_tool = r.last_referenced_tool;
	break;
      case 7:
	r.coolant_setting = COOLANT_MIST;
	break;
      case 8:
	r.coolant_setting = COOLANT_FLOOD;
	break;
      case 9:
	r.coolant_setting = COOLANT_OFF;
	break;
      default:
	cout << "Unhandled word: " << *m << endl;
	assert(false);
      }
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(m)), b.end());
      m = find_icode('M', b);
    }
    
    icode* g = find_icode('G', b);
    while (g != NULL) {
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
      case 17:
	r.active_plane = XY_PLANE;
	break;
      case 28:
	r.active_non_modal_setting = MOVE_HOME_THROUGH_POINT;
	break;
      case 40:
	r.tool_radius_comp = TOOL_RADIUS_COMP_OFF;
	break;
      case 43:
	r.tool_height_comp = TOOL_HEIGHT_COMP_NEGATIVE;
	break;
      case 49:
	r.tool_height_comp = TOOL_HEIGHT_COMP_OFF;
	break;
      case 54:
	r.active_coord_system = G54_COORD_SYSTEM;
	break;
      case 80:
	// TODO: Add support for canned cycles
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
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(g)), b.end());
      g = find_icode('G', b);
    }
    if (b.size() > 0) {
      cout << "Not all instructions in the block were processed: " << b << endl;
      assert(false);
    }
    return r;
  }

  vector<machine_state> all_program_states(const vector<block>& p) {
    vector<machine_state> ms;
    ms.push_back(machine_state());
    for (vector<block>::const_iterator it = p.begin(); it != p.end(); ++it) {
      ms.push_back(next_machine_state(*it, ms.front()));
    }
    return ms;
  }

  bool operator==(const machine_state& l, const machine_state& r) {
    return *(l.feedrate) == *(r.feedrate) &&
      *(l.spindle_speed) == *(r.spindle_speed) &&
      l.active_move_type == r.active_move_type &&
      l.active_coord_system == r.active_coord_system &&
      l.tool_height_comp == r.tool_height_comp &&
      l.tool_radius_comp == r.tool_radius_comp &&
      l.active_plane == r.active_plane &&
      l.active_non_modal_setting == r.active_non_modal_setting &&
      l.spindle_setting == r.spindle_setting &&
      *(l.last_referenced_tool) == *(r.last_referenced_tool) &&
      *(l.active_tool) == *(r.active_tool) &&
      l.coolant_setting == r.coolant_setting;
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

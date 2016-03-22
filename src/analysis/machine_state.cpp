#include <stack>

#include "analysis/machine_state.h"
#include "analysis/utils.h"
#include "core/value.h"

namespace gca {

  struct matches_char {
    char c;
    matches_char(char cp) : c(cp) {}
    bool operator()(const token t) {
      if (t.tp() == ICODE) {
	return t.c == c;
      }
      return false;
    }
  };

  const token* find_icode(char c, const block& b) {
    block::const_iterator i = find_if(b.begin(), b.end(), matches_char(c));
    if (i == b.end()) { return NULL; }
    return &(*i);
  }

  void set_negative_tool_height_comp(machine_state& r, block& b) {
    r.tool_height_comp = TOOL_HEIGHT_COMP_NEGATIVE;
    const token* h = find_icode('H', b);
    assert(h);
    assert(h->v->is_ilit());
    ilit* sr = static_cast<ilit*>(h->v);
    r.tool_height_value = ilit::make(sr->v);
    b.erase(remove_if(b.begin(), b.end(), cmp_token_to(h)), b.end());
  }

  void set_tool_radius_comp_left(machine_state& r, block& b) {
    r.tool_radius_comp = TOOL_RADIUS_COMP_LEFT;
    const token* h = find_icode('H', b);
    if (!h) {
      h = find_icode('D', b);
    }
    if (h) {
      assert(h->v->is_ilit());
      ilit* sr = static_cast<ilit*>(h->v);
      r.tool_radius_value = ilit::make(sr->v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(h)), b.end());
    }
  }

  void set_tool_radius_comp_right(machine_state& r, block& b) {
    r.tool_radius_comp = TOOL_RADIUS_COMP_RIGHT;
    const token* h = find_icode('H', b);
    if (!h) {
      h = find_icode('D', b);
    }
    if (h) {
      assert(h->v->is_ilit());
      ilit* sr = static_cast<ilit*>(h->v);
      r.tool_radius_value = ilit::make(sr->v);
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(h)), b.end());
    }
  }
  
  bool no_state_effect(const token& t) {
    if (t.tp() == ICODE) {
      return t.c == 'N' || t.c == 'O' ||
	t.c == 'n' || t.c == 'o';
    }
    return true;
  }

  void update_m_codes(block& b, machine_state& r) {
    const token* m = find_icode('M', b);
    while (m != NULL) {
      assert(m->v->is_ilit());
      ilit* sr = static_cast<ilit*>(m->v);
      switch (sr->v) {
      case 0:
	break;
      case 1:
	break;
      case 2:
	break;
      case 3:
	r.spindle_setting = SPINDLE_CLOCKWISE;
	break;
      case 4:
	r.spindle_setting = SPINDLE_COUNTERCLOCKWISE;
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
      case 19:
	// TODO: Figure out what this instruction actually does
	break;
      case 30:
	break;
      default:
	cout << "Unhandled word: " << *m << endl;
	assert(false);
      }
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(m)), b.end());
      m = find_icode('M', b);
    }
  }

  void update_g_codes(block& b, machine_state& r) {
    const token* g = find_icode('G', b);
    while (g != NULL) {
      assert(g->v->is_ilit());
      ilit* sr = static_cast<ilit*>(g->v);
      switch(sr->v) {
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
      case 18:
	r.active_plane = ZX_PLANE;
	break;
      case 19:
	r.active_plane = YZ_PLANE;
	break;
      case 28:
	r.active_non_modal_setting = MOVE_HOME_THROUGH_POINT;
	break;
      case 40:
	r.tool_radius_comp = TOOL_RADIUS_COMP_OFF;
	break;
      case 41:
	set_tool_radius_comp_left(r, b);
	break;
      case 42:
	set_tool_radius_comp_right(r, b);
	break;
      case 43:
	set_negative_tool_height_comp(r, b);
	break;
      case 49:
	r.tool_height_comp = TOOL_HEIGHT_COMP_OFF;
	break;
      case 53:
	r.active_non_modal_setting = POSITION_REGISTER_MOVE;
	break;
      case 54:
	r.active_coord_system = G54_COORD_SYSTEM;
	break;
      case 73:
	r.active_move_type = CANNED_CYCLE_81_MOVE;
	break;
      case 80:
	r.active_move_type = CANCEL_CANNED_CYCLE_MOVE;
	break;
      case 81:
	r.active_move_type = CANNED_CYCLE_81_MOVE;
	break;
      case 83:
	r.active_move_type = CANNED_CYCLE_83_MOVE;
	break;
      case 84:
	r.active_move_type = CANNED_CYCLE_84_MOVE;
	break;
      case 85:
	r.active_move_type = CANNED_CYCLE_85_MOVE;
	break;
      case 90:
	r.active_distance_mode = ABSOLUTE_DISTANCE_MODE;
	break;
      case 91:
	r.active_distance_mode = RELATIVE_DISTANCE_MODE;
	break;
      case 92:
	r.active_non_modal_setting = POSITION_REGISTER_MOVE;
	break;
      case 98:
	r.active_canned_cycle_return_policy = CANNED_CYCLE_RETURN_Z;
	break;
      case 99:
	r.active_canned_cycle_return_policy = CANNED_CYCLE_RETURN_R;
	break;
      default:
	cout << "Unsupported g instruction: " << *g << endl;
	assert(false);
      }
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(g)), b.end());
      g = find_icode('G', b);
    }
  }

  value* replace_value(value* old, char c, block& b) {
    const token* f = find_icode(c, b);
    value* l;
    if (f) {
      if (f->v->is_lit()) {
	lit* fr = static_cast<lit*>(f->v);
	l = lit::make(fr->v);
      } else if (f->v->is_ilit()) {
	ilit* fr = static_cast<ilit*>(f->v);
	l = ilit::make(fr->v);
      } else {
	assert(false);
      }
      b.erase(remove_if(b.begin(), b.end(), cmp_token_to(f)), b.end());
      return l;
    }
    return old;
  }

  int line_no(const block& b) {
    //cout << "Block: " << b << endl;
    if (b.size() == 0) {
      return -1;
    }
    return b.front().line_no;
  }
  
  machine_state next_machine_state(const block& bs, const machine_state& s) {
    block b = bs;
    b.erase(remove_if(b.begin(), b.end(), no_state_effect), b.end());
    machine_state r = s;
    r.line_no = line_no(b);
    r.feedrate = replace_value(r.feedrate, 'F', b);
    r.spindle_speed = replace_value(r.spindle_speed, 'S', b);
    value* om = omitted::make();
    r.x = replace_value(om, 'X', b);
    r.y = replace_value(om, 'Y', b);
    r.z = replace_value(om, 'Z', b);
    r.i = replace_value(om, 'I', b);
    r.j = replace_value(om, 'J', b);
    r.k = replace_value(om, 'K', b);
    r.last_referenced_tool = replace_value(r.last_referenced_tool, 'T', b);
    r.r = replace_value(r.r, 'R', b);
    r.q = replace_value(r.k, 'Q', b);
    r.active_non_modal_setting = NO_NON_MODAL_SETTING;

    update_m_codes(b, r);
    update_g_codes(b, r);
    if (b.size() > 0) {
      cout << "Not all instructions in the block were processed: " << b << endl;
      assert(false);
    }
    return r;
  }

  vector<machine_state> all_program_states(const machine_state& init,
					   const vector<block>& p) {
    vector<machine_state> ms;
    ms.push_back(init);
    vector<pair<token, program_loc> > subroutine_starts = compute_starts(p);
    stack<vector<block>::const_iterator> istack;
    vector<block>::const_iterator it = p.begin();
    while (it < p.end()) {
      const block& b = *it;
      if (is_call_block(b)) {
	++it;
	istack.push(it);
	it = find_called_subroutine(b, subroutine_starts);
      } else if (is_ret_block(*it)) {
	it = istack.top();
	istack.pop();
      } else if (is_end_block(*it)) {
	ms.push_back(next_machine_state(*it, ms.back()));
	break;
      } else {
	ms.push_back(next_machine_state(*it, ms.back()));
	++it;
      }
    }
    return ms;
  }
  
  
  vector<machine_state> all_program_states(const vector<block>& p) {
    machine_state ms;
    return all_program_states(ms, p);
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
      l.coolant_setting == r.coolant_setting &&
      *(l.tool_height_value) == *(r.tool_height_value) &&
      *(l.x) == *(r.x) && *(l.y) == *(r.y) && *(l.z) == *(r.z) &&
      *(l.i) == *(r.i) && *(l.j) == *(r.j) && *(l.k) == *(r.k) &&
      *(l.tool_radius_value) == *(r.tool_radius_value);
  }

  bool operator!=(const machine_state& l, const machine_state& r)
  { return !(l == r); }

  void print_val(ostream& stream, const value* v) {
    if (v->is_omitted()) {
      stream << "<omitted>";
    } else {
      stream << *v;
    }
  }

  ostream& operator<<(ostream& stream, const machine_state& s) {
    stream << "F ";
    print_val(stream, s.feedrate);
    stream << endl;
    stream << "T ";
    print_val(stream, s.last_referenced_tool);
    stream << endl;
    stream << "X ";
    print_val(stream, s.x);
    stream << endl;
    stream << "Y ";
    print_val(stream, s.y);
    stream << endl;
    stream << "Z ";
    print_val(stream, s.z);
    stream << endl;
    stream << "I ";
    print_val(stream, s.i);
    stream << endl;
    stream << "J ";
    print_val(stream, s.j);
    stream << endl;
    stream << "K ";
    print_val(stream, s.k);
    stream << endl;
    stream << s.spindle_setting << endl;
    stream << s.active_move_type << endl;
    return stream;
  }

  ostream& operator<<(ostream& out, const spindle_state s) {
    if (s == SPINDLE_STATE_UNKNOWN) {
      out << "SPINDLE_STATE_UNKNOWN";
    } else if (s == SPINDLE_OFF) {
      out << "SPINDLE_OFF";
    } else if (s == SPINDLE_CLOCKWISE) {
      out << "SPINDLE_CLOCKWISE";
    } else if (s == SPINDLE_COUNTERCLOCKWISE) {
      out << "SPINDLE_COUNTERCLOCKWISE";
    } else {
      assert(false);
    }
    return out;
  }

  ostream& operator<<(ostream& out, const move_type s) {
    if (s == UNKNOWN_MOVE_TYPE) {
      out << "UNKNOWN_MOVE_TYPE";
    } else if (s == FAST_MOVE) {
      out << "FAST_MOVE";
    } else if (s == LINEAR_MOVE) {
      out << "LINEAR_MOVE";
    } else if (s == CLOCKWISE_CIRCULAR_MOVE) {
      out << "CLOCKWISE_CIRCULAR_MOVE";
    } else if (s == COUNTERCLOCKWISE_CIRCULAR_MOVE) {
      out << "COUNTERCLOCKWISE_CIRCULAR_MOVE";
    } else if (s == CANCEL_CANNED_CYCLE_MOVE) {
      out << "CANCEL_CANNED_CYCLE_MOVE";
    } else if (s == CANNED_CYCLE_73_MOVE) {
      out << "CANNED_CYCLE_73_MOVE";
    } else if (s == CANNED_CYCLE_81_MOVE) {
      out << "CANNED_CYCLE_81_MOVE";
    } else if (s == CANNED_CYCLE_83_MOVE) {
      out << "CANNED_CYCLE_83_MOVE";
    } else if (s == CANNED_CYCLE_84_MOVE) {
      out << "CANNED_CYCLE_84_MOVE";
    } else if (s == CANNED_CYCLE_85_MOVE) {
      out << "CANNED_CYCLE_85_MOVE";
    } else {
      assert(false);
    }
    return out;
  }

  ostream& operator<<(ostream& out, const coord_system s) {
    switch (s) {
    case UNKNOWN_COORD_SYSTEM:
      out << "UNKNOWN_COORD_SYSTEM";
      break;
    case MACHINE_COORD_SYSTEM:
      out << "MACHINE_COORD_SYSTEM";
      break;
    case G54_COORD_SYSTEM:
      out << "G54_COORD_SYSTEM";
      break;
    default:
      assert(false);
    }
    return out;
  }

  ostream& operator<<(ostream& stream, const vector<machine_state>& s) {
    for (auto m : s) {
      stream << m << endl;
    }
    return stream;
  }
}

#include "analysis/position_table.h"
#include "analysis/utils.h"
#include "system/algorithm.h"

namespace gca {

  position unknown_pos() {
    return position(omitted::make(), omitted::make(), omitted::make());
  }
  
  vector<coord_system> all_coord_systems() {
    vector<coord_system> acs;
    acs.push_back(UNKNOWN_COORD_SYSTEM);
    acs.push_back(G54_COORD_SYSTEM);
    acs.push_back(MACHINE_COORD_SYSTEM);
    return acs;
  }

  position last_position(coord_system c, const position_table& t) {
    assert(t.size() > 0);
    position_table_row r = t.back();
    for (auto e : r) {
      if (e.first == c) { return e.second; }
    }
    assert(false);
  }

  position_table_row unknown_row() {
    vector<coord_system> cs = all_coord_systems();
    position_table_row r;
    for (auto c : cs) {
      r.push_back(position_entry(c, unknown_pos()));
    }
    return r;
  }

  void add_row(position_table_row r, position_table& x) {
    x.push_back(r);
  }

  void add_unk_row(position_table& x) {
    add_row(unknown_row(), x);
  }

  void copy_last_row(position_table& t) {
    assert(t.size() > 0);
    t.push_back(t.back());
  }
  
  bool operator==(const position& l, const position r)
  { return (*(l.x) == *(r.x)) && (*(l.y) == *(r.y)) && ((*l.z) == *(r.z)); }
  
  bool operator==(const position_entry& x, const position_entry& y)
  { return (x.first == y.first) && (x.second == y.second); }

  bool operator==(const position_table_row& x, const position_table_row& y) {
    if (x.size() != y.size()) { return false; }
    return equal(x.begin(), x.end(), y.begin());
  }

  bool operator==(const position_table& x, const position_table& y) {
    if (x.size() != y.size()) { return false; }
    return equal(x.begin(), x.end(), y.begin());
  }

  bool operator!=(const position_table& x, const position_table& y)
  { return !(x == y); }

  void update_table(coord_system c, const position p, position_table& t) {
    vector<coord_system> cs = all_coord_systems();
    position_table_row r;
    for (auto cs : cs) {
      if (cs != c) {
	r.push_back(position_entry(cs, unknown_pos()));
      } else {
	r.push_back(position_entry(cs, position(p)));
      }
    }
    add_row(r, t);
  }

  value* increment_value(value* v, value* inc) {
    if (v->is_omitted()) {
      return v;
    } else if (v->is_lit() && inc->is_lit()) {
      lit* vl = static_cast<lit*>(v);
      lit* incl = static_cast<lit*>(inc);
      return lit::make(vl->v + incl->v);
    } else if (v->is_lit() && inc->is_omitted()) {
      return v;
    } else {
      cout << "V = " << *v << endl;
      cout << "Inc = " << *inc << endl;
      assert(false);
    }
  }
  
  position increment_position(const position p, const position inc) {
    return position(increment_value(p.x, inc.x),
		    increment_value(p.y, inc.y),
		    increment_value(p.z, inc.z));
  }

  position next_relative_position(const machine_state& s,
				  const position_table& t) {
    assert(t.size() > 0);
    position last = last_position(s.active_coord_system, t);
    position inc(s.x, s.y, s.z);
    return increment_position(last, inc);
  }

  void update_table_with(position_table& t,
			 const machine_state& s,
			 const machine_state& next) {
    if (s.active_non_modal_setting == MOVE_HOME_THROUGH_POINT) {
      update_table(MACHINE_COORD_SYSTEM, position(0.0, 0.0, 0.0), t);
    } else if (is_move(s)) {
      if (s.active_distance_mode == ABSOLUTE_DISTANCE_MODE) {
	value* x = s.x->is_omitted() ? last_position(s.active_coord_system, t).x : s.x;
	value* y = s.y->is_omitted() ? last_position(s.active_coord_system, t).y : s.y;
	value* z = s.z->is_omitted() ? last_position(s.active_coord_system, t).z : s.z;
	update_table(s.active_coord_system, position(x, y, z), t);
      } else if (s.active_distance_mode == RELATIVE_DISTANCE_MODE) {
	position p = next_relative_position(s, t);
	update_table(s.active_coord_system, p, t);
      } else {
	cout << "Error: No distance mode set" << endl;
	assert(false);
      }
    } else if (s.active_tool != next.active_tool) {
      add_unk_row(t);
    } else {
      if (t.size() == 0) {
	add_unk_row(t);
      } else {
	copy_last_row(t);
      }
    }
  }

  position_table program_position_table(const vector<machine_state>& p) {
    position_table t;
    if (p.size() > 0) { add_unk_row(t); }
    apply_between(p.begin(), p.end(),
		  [&t](const machine_state& previous, const machine_state& next)
		  { update_table_with(t, next, previous); });
    return t;
  }

  ostream& operator<<(ostream& out, const position_entry& e) {
    out << "(" << e.first << ", ";
    out << *(e.second.x) << ", ";
    out << *(e.second.y) << ", ";
    out << *(e.second.z);
    out << ")";
    return out;
  }

  ostream& operator<<(ostream& out, const position_table& t) {
    for (auto r : t) {
      for (auto e : r)
	{ out << "| " << e.first << " " << e.second << " "; }
      out << endl;
    }
    return out;
  }

  ostream& operator<<(ostream& out, const position& p) {
    out << "(" << *(p.x) << ", " << *(p.y) << ", " << *(p.z) << ")";
    return out;
  }

  ostream& operator<<(ostream& out, const vector<position>& ps) {
    for (auto p : ps) {
      out << p << endl;
    }
    return out;
  }

  vector<position> select_column(coord_system c, const position_table& t) {
    vector<position> ps;
    for (auto r : t) {
      auto e = *find_if(r.begin(), r.end(),
			[c](const position_entry& et)
			{ return et.first == c; });
      ps.push_back(e.second);
    }
    return ps;
  }
}

#include "analysis/position_table.h"
#include "analysis/utils.h"

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

  position_table_row unknown_row() {
    vector<coord_system> cs = all_coord_systems();
    position_table_row r;
    for (vector<coord_system>::iterator it = cs.begin(); it != cs.end(); ++it) {
      r.push_back(position_entry(*it, unknown_pos()));
    }
    return r;
  }

  void add_row(position_table_row r, position_table& x) {
    x.push_back(r);
  }

  void add_unk_row(position_table& x) {
    add_row(unknown_row(), x);
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
    for (vector<coord_system>::iterator it = cs.begin(); it != cs.end(); ++it) {
      coord_system cs = *it;
      if (cs != c) {
	r.push_back(position_entry(cs, unknown_pos()));
      } else {
	r.push_back(position_entry(cs, position(p)));
      }
    }
    add_row(r, t);
  }
  
  position_table program_position_table(const vector<machine_state>& p) {
    position_table_row last = unknown_row();
    position_table t;    
    for (vector<machine_state>::const_iterator it = p.begin() + 1;
	 it < p.end(); ++it) {
      machine_state s = *it;
      if (is_move(s)) {
	update_table(s.active_coord_system, position(s.x, s.y, s.z), t);
      } else {
	add_unk_row(t);
      }
    }
    return t;
  }

}

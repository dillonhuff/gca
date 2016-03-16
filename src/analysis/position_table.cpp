#include "analysis/position_table.h"

namespace gca {

  vector<coord_system> all_coord_systems() {
    vector<coord_system> acs;
    acs.push_back(UNKNOWN_COORD_SYSTEM);
    acs.push_back(G54_COORD_SYSTEM);
    acs.push_back(MACHINE_COORD_SYSTEM);
    return acs;
  }

  bool operator==(const position& l, const position r)
  { return (*(l.x) == *(r.x)) && (*(l.y) == *(r.y)) && ((*l.z) == *(r.z)); }
  
  bool operator==(const position_entry& x, const position_entry& y) {
    position l = x.second;
    position r = y.second;
    return (x.first == y.first) && (l == r); // && (x.second == y.second);
  }

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
  
  position_table program_position_table(const vector<machine_state>& p) {
    //position_table_row r = unknown_row();
    position_table t;    
    for (vector<machine_state>::const_iterator it = p.begin() + 1;
	 it < p.end(); ++it) {
      add_unk_row(t);
    }
    return t;
  }

  position unknown_pos() {
    return position(omitted::make(), omitted::make(), omitted::make());
  }

  void add_unk_row(position_table& x) {
    vector<coord_system> cs = all_coord_systems();
    position_table_row r;
    for (vector<coord_system>::iterator it = cs.begin(); it != cs.end(); ++it) {
      r.push_back(position_entry(*it, unknown_pos()));
    }
    x.push_back(r);
  }
}

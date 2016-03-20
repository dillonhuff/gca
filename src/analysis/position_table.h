#ifndef GCA_POSITION_TABLE_H
#define GCA_POSITION_TABLE_H

#include <utility>
#include <vector>

#include "analysis/machine_state.h"

using namespace std;

namespace gca {

  struct position {
    value* x;
    value* y;
    value* z;
    position(value* xp, value* yp, value* zp) : x(xp), y(yp), z(zp) {}
    position(const position& p) : x(p.x), y(p.y), z(p.z) {}
    position(double xp, double yp, double zp) :
      x(lit::make(xp)), y(lit::make(yp)), z(lit::make(zp)) {}
  };

  typedef pair<coord_system, position> position_entry;

  typedef vector<position_entry> position_table_row;

  typedef vector<position_table_row> position_table;

  void update_table(coord_system c, const position p, position_table& t);
  bool operator==(const position_table& x, const position_table& y);
  bool operator!=(const position_table& x, const position_table& y);
  position_table program_position_table(const vector<machine_state>& p);
  void add_unk_row(position_table& x);

  ostream& operator<<(ostream& out, const position_entry& e);
  ostream& operator<<(ostream& out, const position_table& e);
}

#endif 

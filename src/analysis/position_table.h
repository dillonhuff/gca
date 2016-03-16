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
  };

  typedef pair<coord_system, position> position_entry;

  typedef vector<position_entry> position_table_row;

  typedef vector<vector<pair<coord_system, position> > > position_table;

  bool operator==(const position_table& x, const position_table& y);
  bool operator!=(const position_table& x, const position_table& y);
  position_table program_position_table(const vector<machine_state>& p);
  void add_unk_row(position_table& x);
}

#endif 

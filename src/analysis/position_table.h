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
    position() : x(omitted::make()), y(omitted::make()), z(omitted::make()) {}
    position(value* xp, value* yp, value* zp) : x(xp), y(yp), z(zp) {}
    position(const position& p) : x(p.x), y(p.y), z(p.z) {}
    position(double xp, double yp, double zp) :
      x(lit::make(xp)), y(lit::make(yp)), z(lit::make(zp)) {}

    inline bool is_lit() const
    { return x->is_lit() && y->is_lit() && z->is_lit(); }

    inline point extract_point() const {
      assert(is_lit());
      lit* xl = static_cast<lit*>(x);
      lit* yl = static_cast<lit*>(y);
      lit* zl = static_cast<lit*>(z);
      return point(xl->v, yl->v, zl->v);
    }

  };

  typedef pair<coord_system, position> position_entry;

  typedef vector<position_entry> position_table_row;

  typedef vector<position_table_row> position_table;

  void update_table(coord_system c, const position p, position_table& t);
  bool operator==(const position_table& x, const position_table& y);
  bool operator!=(const position_table& x, const position_table& y);
  position_table program_position_table(const vector<machine_state>& p);
  void add_unk_row(position_table& x);
  vector<position> select_column(coord_system c, const position_table& t);

  ostream& operator<<(ostream& out, const position& p);
  ostream& operator<<(ostream& out, const position_entry& e);
  ostream& operator<<(ostream& out, const position_table& e);
  ostream& operator<<(ostream& out, const vector<position>& e);
}

#endif 

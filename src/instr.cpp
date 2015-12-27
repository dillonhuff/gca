#include "instr.h"

namespace gca {

  bool instr::operator==(const instr& other) {
    if (c != other.c || v != other.v) {
      return false;
    }
    if (c == GCA_G && (v == 0 || v == 1)) {
      point lp = pos();
      point op = other.pos();
      bool res = within_eps(lp, op) && feed_rate == other.feed_rate && orient == other.orient;
      if (!(orient == other.orient)) {
	cout << "!ORIENT " << *this << " != " << other << endl;
      }
      if (!(feed_rate == other.feed_rate)) {
	cout << "!FEED RATE " << *this << " != " << other << endl;
      }
      if (!within_eps(lp, op)) {
	cout << "!within EPS " << *this << " != " << other << endl;
      }
      if (!res) {
	cout << *this << " != " << other << endl;
      }
      return res;
    }
    return true;
  }
  
  void instr::print(ostream& s) const {
    if (c == GCA_M) {
      cout << 'M' << v;
    } else if (c == GCA_G && (v == 0 || v == 1)) {
      cout << 'G' << v;
      if (v != 0) {
	cout << " F" << feed_rate;
      }
      s << " X" << pos().x;
      s << " Y" << pos().y;
      s << " Z" << pos().z;
      if (is_rel()) {
	cout << " ( relative )";
      } else {
	cout << " ( absolute )";
      }
    } else if (is_G()) {
      assert(orient == GCA_NONE && orient != GCA_RELATIVE && orient != GCA_ABSOLUTE);
      cout << 'G' << v;
    } else {
      assert(false);
    }
  }

  ostream& operator<<(ostream& stream, const instr& i) {
    i.print(stream);
    return stream;
  }

}

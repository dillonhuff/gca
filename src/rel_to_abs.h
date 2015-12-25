#ifndef GCA_REL_TO_ABS
#define GCA_REL_TO_ABS

namespace gca {

  class rel_to_abs {
  public:
    gprog* apply(context& c, gprog* p) const {
      vector<point> positions = p->all_positions_starting_at(point(0, 0, 0));
      gprog* n = c.mk_gprog();
      for (int i = 1; i < p->size() + 1; i++) {
	instr* ist = (*p)[i-1];
	if (ist->is_G()) {
	  point pos = positions[i];
	  instr* next_ist;
	  if (ist->v == 0) {
	    next_ist = c.mk_G0(pos, GCA_ABSOLUTE);
	  } else if (ist->v == 1) {
	    next_ist = c.mk_G1(pos.x, pos.y, pos.z, ist->feed_rate, GCA_ABSOLUTE);
	  } else {
	    assert(false);
	  }
	  assert(next_ist->is_abs());
	  n->push_back(next_ist);
	} else {
	  n->push_back(ist);
	}
      }
      return n;
    }

  };
}

#endif

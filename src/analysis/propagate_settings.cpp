#include "analysis/propagate_settings.h"

namespace gca {

  vector<token*> mode_conflicts(const token& t) {
    vector<token*> cs;
    if (t.tp() != ICODE) { return cs; }
    const icode& ic = static_cast<const icode&>(t);
    if (ic.c == 'G' && ic.v == ilit(90)) {
      cs.push_back(icode::make('G', 91));
    } else if (ic.c == 'G' && ic.v == ilit(41)) {
      cs.push_back(icode::make('G', 40));
      cs.push_back(icode::make('G', 42));
    } else if (ic.c == 'G' && ic.v == ilit(0)) {
      cs.push_back(icode::make('G', 1));
    } else {
      cout << "mode_conflicts cannot handle: " << ic << endl;
      assert(false);
    }
    return cs;
  }

  struct elem_of {
    const vector<token*>& conflicts;
    elem_of(const vector<token*>& conflictsp) : conflicts(conflictsp) {}
    bool operator()(const token* t) {
      return find_if(conflicts.begin(), conflicts.end(), cmp_token_to(t)) != conflicts.end();
    }
  };

  struct settings_conflicts {
    const block& c;
    settings_conflicts(const block& cp) : c(cp) {}
    bool operator()(const token* t) {
      vector<token*> conflicts = mode_conflicts(*t);
      return find_first_of(c.begin(), c.end(),
			   conflicts.begin(), conflicts.end()) != c.end();
    }
  };

  bool non_modal(const token* t) {
    if (t->tp() != ICODE) { return true; }
    const icode* ic = static_cast<const icode*>(t);
    if (ic->c == 'X' || ic->c == 'Y' || ic->c == 'Z') {
      return true;
    }
    return false;
  }

  vector<block> propagate_settings(const vector<block>& p) {
    vector<block> bs;
    block last;
    for (vector<block>::const_iterator it = p.begin(); it != p.end(); ++it) {
      block b = last;
      b.erase(remove_if(b.begin(), b.end(), non_modal), b.end());
      block current = *it;
      b.erase(remove_if(b.begin(), b.end(), settings_conflicts(current)), b.end());
      b.insert(b.end(), current.begin(), current.end());
      bs.push_back(b);
      last = current;
    }
    return bs;
  }
  
}

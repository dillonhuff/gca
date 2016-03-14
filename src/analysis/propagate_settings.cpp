#include "analysis/propagate_settings.h"

namespace gca {

  // vector<vector<tok> > all_modes() {
  //   vector<vector<tok> > modes;
  //   vector<tok> offset;
  //   offset.push_back(new icode('G', *(new ilit(90))));
  //   offset.push_back(new icode('G', *(new ilit(91))));
  //   vector<tok> tool_radius_comp;
  //   tool_radius_comp.push_back(new icode('G', *(new ilit(40))));
  //   tool_radius_comp.push_back(new icode('G', *(new ilit(41))));
  //   tool_radius_comp.push_back(new icode('G', *(new ilit(42))));
  //   vector<tok> tool_length_comp;
  //   tool_length_comp.push_back(new icode('G', *(new ilit(43))));
  //   tool_length_comp.push_back(new icode('G', *(new ilit(44))));
  //   tool_length_comp.push_back(new icode('G', *(new ilit(49))));
  //   vector<tok> move_type;
  //   move_type.push_back(new icode('G', *(new ilit(0))));
  //   move_type.push_back(new icode('G', *(new ilit(1))));
  //   vector<tok> coords;
  //   coords.push_back(new icode('G', *(new ilit(54))));
  //   coords.push_back(new icode('G', *(new ilit(55))));
  //   coords.push_back(new icode('G', *(new ilit(56))));
  //   coords.push_back(new icode('G', *(new ilit(57))));
  //   coords.push_back(new icode('G', *(new ilit(58))));
  //   coords.push_back(new icode('G', *(new ilit(59))));
  //   vector<tok> plane_selection;
  //   plane_selection.push_back(new icode('G', *(new ilit(17))));
  //   plane_selection.push_back(new icode('G', *(new ilit(18))));
  //   plane_selection.push_back(new icode('G', *(new ilit(19))));
  //   vector<tok> coolant_setting;
  //   //coolant_setting.push_back(new icode(''));
  //   modes.push_back(offset);
  //   modes.push_back(tool_radius_comp);
  //   modes.push_back(tool_length_comp);
  //   modes.push_back(move_type);
  //   modes.push_back(coords);
  //   modes.push_back(plane_selection);
  //   return modes;
  // }

  // static vector<vector<tok> > modes = all_modes();

  // struct contains {
  //   token& t;
  //   contains(token& tp) : t(tp) {}
  //   bool operator()(vector<tok>& mode)
  //   { return find_if(mode.begin(), mode.end(), cmp_token_to(&t)) != mode.end(); }
  // };

  // vector<tok> mode_conflicts(token& t) {
  //   vector<vector<tok> >::iterator cs =
  //     find_if(modes.begin(), modes.end(), contains(t));
  //   if (cs == modes.end()) {
  //     cout << "Unsupported instruction: " << t << endl;
  //     assert(false);
  //   }
  //   return *cs;
  // }

  // struct elem_of {
  //   const vector<tok>& conflicts;
  //   elem_of(const vector<tok>& conflictsp) : conflicts(conflictsp) {}
  //   bool operator()(const tok t) {
  //     return find_if(conflicts.begin(), conflicts.end(), cmp_token_to(t)) != conflicts.end();
  //   }
  // };

  // struct settings_conflicts {
  //   const block& c;
  //   settings_conflicts(const block& cp) : c(cp) {}
  //   bool operator()(const tok t) {
  //     vector<tok> conflicts = mode_conflicts(*t);
  //     return find_first_of(c.begin(), c.end(),
  // 			   conflicts.begin(), conflicts.end()) != c.end();
  //   }
  // };

  // bool non_modal(const tok t) {
  //   if (t->tp() != ICODE) { return true; }
  //   const icode* ic = static_cast<const icode*>(t);
  //   if (ic->c == 'X' ||
  // 	ic->c == 'Y' ||
  // 	ic->c == 'Z' ||
  // 	ic->c == 'O' ||
  // 	ic->c == 'N') {
  //     return true;
  //   }
  //   if (ic->c == 'G' && ic->v == ilit(80)) { return true; }
  //   if (ic->c == 'G' && ic->v == ilit(28)) { return true; }
  //   return false;
  // }

  vector<block> propagate_settings(const vector<block>& p) {
    vector<block> bs;
    // block last;
    // for (vector<block>::const_iterator it = p.begin(); it != p.end(); ++it) {
    //   block b = last;
    //   b.erase(remove_if(b.begin(), b.end(), non_modal), b.end());
    //   block current = *it;
    //   b.erase(remove_if(b.begin(), b.end(), settings_conflicts(current)), b.end());
    //   b.insert(b.end(), current.begin(), current.end());
    //   bs.push_back(b);
    //   last = current;
    // }
    return bs;
  }

}

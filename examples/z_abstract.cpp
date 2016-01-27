#include <cassert>
#include <iostream>
#include <map>
#include <set>

#include "core/context.h"
#include "core/parser.h"

using namespace gca;

void find_z_values(set<double>& z_values, gprog* p) {
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    if (is->is_move_instr()) {
      move_instr* mi = static_cast<move_instr*>(is);
      value* z = mi->get_z();
      if (z->is_lit()) {
	double z_val = static_cast<lit*>(z)->v;
	if (z_values.find(z_val) == z_values.end()) {
	  z_values.insert(z_val);
	}
      }
    }
  }
}

gprog* generalize_zs(set<double>& z_values, gprog* p) {
  gprog* r = mk_gprog();
  map<double, var*> new_vars;
  int i = 1;
  for (set<double>::iterator it = z_values.begin();
       it != z_values.end(); ++it) {
    var* v = mk_var(i);
    new_vars[*it] = mk_var(i);
    r->push_back(mk_assign(v, mk_lit(*it)));
    i++;
  }
  for (int j = 0; j < p->size(); j++) {
    instr *is = (*p)[j];
    if (is->is_move_instr()) {
      move_instr* mi = static_cast<move_instr*>(is);
      value* z = mi->get_z();
      if (z->is_lit()) {
	double z_val = static_cast<lit*>(z)->v;
	var* l = new_vars[z_val];
	move_instr* new_mi = static_cast<move_instr*>(mk_instr_cpy(mi));
	new_mi->set_z(l);
	r->push_back(new_mi);
      } else {
	r->push_back(is);
      }
    } else {
      r->push_back(is);
    }
  }
  return r;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: gdiff <gcode_file_path>" << endl;
    return 0;
  }
  string file = argv[1];
  
  gprog* p = read_file(file);
  set<double> z_values;
  find_z_values(z_values, p);
  gprog* r = generalize_zs(z_values, p);
  cout << *r;
  return 0;
}

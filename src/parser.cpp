#include <cctype>
#include <iostream>

#include "src/parser.h"

namespace gca {

  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    for (string::size_type i = 0; i < s.size(); i++) {
      if (s[i] == 'M') {
	i++;
      	int val = stoi(s.substr(i), &i);
      	p->push_back(c.mk_minstr(val));
      } if (s[i] == 'G') {
	i++;
	int val = stoi(s.substr(i), &i);
	if (val == 0) {
	  
	} else {
	  assert(false);
	}
      }
    }
    return p;
  }
  
}

#include <cctype>
#include <iostream>

#include "src/parser.h"

namespace gca {

  int parse_int(string::iterator& it,
		string::iterator end) {
    string digits = "";
    while (it != end && isdigit(*it)) {
      digits += *it;
      it++;
    }
    return stoi(digits);
  }

  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    for (string::iterator it = s.begin();
	 it != s.end(); ++it) {
      if (*it == 'M') {
      	it++;
      	int val = parse_int(it, s.end());
      	p->push_back(c.mk_minstr(val));
	if (it == s.end()) {
	  break;
	}	
      }
    }
    return p;
  }
  
}

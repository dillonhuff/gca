#include <cctype>
#include <fstream>
#include <iostream>
#include <streambuf>

#include "parser.h"

namespace gca {

  void ignore_whitespace(size_t* i, string s) {
    while (*i < s.size() && isspace(s[*i])) {
      (*i)++;
    }
  }

  double parse_double(size_t* i, string s) {
    size_t j = *i;
    double v = stod(s.substr(*i), &j);
    *i += j;
    return v;
  }

  int parse_int(size_t* i, string s) {
    size_t j = *i;
    int v = stoi(s.substr(*i), &j);
    *i += j;
    return v;
  }
  
  double parse_option_coordinate(char c, size_t* i, string s, double def=0.0) {
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      return parse_double(i, s);
    }
    return def;
  }

  instr* parse_G1(context& c, size_t* i, string s) {
    double default_feedrate = 1.0;
    double fr = parse_option_coordinate('F', i, s, default_feedrate);
    double x = parse_option_coordinate('X', i, s);
    double y = parse_option_coordinate('Y', i, s);
    double z = parse_option_coordinate('Z', i, s);
    // TODO: Replace w/ epsilon comparison since these are floating point
    if (fr == default_feedrate) {
      fr = parse_option_coordinate('F', i, s, default_feedrate);
    }
    return c.mk_G1(x, y, z, fr);
  }

  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    string::size_type i = 0;
    while ( i < s.size()) {
      ignore_whitespace(&i, s);
      if (i >= s.size()) { break; }
      if (s[i] == 'M') {
	i++;
	int val = parse_int(&i, s);
      	p->push_back(c.mk_minstr(val));
      } else if (s[i] == 'G') {
	i++;
	int val = parse_int(&i, s);
	if (val == 0) {
	  double x = parse_option_coordinate('X', &i, s);
	  double y = parse_option_coordinate('Y', &i, s);
	  double z = parse_option_coordinate('Z', &i, s);
	  p->push_back(c.mk_G0(x, y, z));
	} else if (val == 1) {
	  instr* is = parse_G1(c, &i, s);
	  p->push_back(is);//c.mk_G1(x, y, z));
	} else {
	  cout << "Unrecognized instr code for instr letter: " << val << endl;
	  assert(false);
	}
	ignore_whitespace(&i, s);
      } else {
	cout << "Cannot parse string: " << s.substr(i) << endl;
	assert(false);
      }
    }
    return p;
  }

  gprog* read_file(context& c, string file_name) {
    ifstream t(file_name);
    string str((istreambuf_iterator<char>(t)),
	       istreambuf_iterator<char>());
    return parse_gprog(c, str);
  }

}

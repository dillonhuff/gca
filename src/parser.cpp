#include <cctype>
#include <fstream>
#include <iostream>
#include <streambuf>

#include "parser.h"

namespace gca {

  void ignore_comment(size_t* i, string s) {
    if (s[*i] == '(') {
      while (*i < s.size() && s[*i] != ')') { (*i)++; }
      (*i)++;
    }
  }

  void ignore_whitespace(size_t* i, string s) {
    while (*i < s.size() && (s[*i] == '(' || isspace(s[*i]))) {
      while (*i < s.size() && isspace(s[*i])) { (*i)++; }
      if (*i >= s.size()) { break; }
      ignore_comment(i, s);
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

  double parse_coordinate(char c, size_t* i, string s) {
    ignore_whitespace(i, s);
    assert(s[*i] == c);
    (*i)++;
    return parse_double(i, s);
  }

  double parse_option_coordinate(char c, size_t* i, string s, double def=0.0) {
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      return parse_double(i, s);
    }
    return def;
  }

  instr* parse_G1(context& c, gprog* p, size_t* i, string s) {
    double default_feedrate = 1.0;
    double fr = parse_option_coordinate('F', i, s, default_feedrate);
    double x, y, z;
    if (p->size() > 0) {
      point lp = p->last_position();
      x = parse_option_coordinate('X', i, s, lp.x);
      y = parse_option_coordinate('Y', i, s, lp.y);
      z = parse_option_coordinate('Z', i, s, lp.z);
    } else {
      x = parse_coordinate('X', i, s);
      y = parse_coordinate('Y', i, s);
      z = parse_coordinate('Z', i, s);
    }
    // TODO: Replace w/ epsilon comparison since these are floating point
    if (fr == default_feedrate) {
      fr = parse_option_coordinate('F', i, s, default_feedrate);
    }
    return c.mk_G1(x, y, z, fr);
  }

  instr* parse_G0(context& c, gprog* p, size_t* i, string s) {
    double x, y, z;
    if (p->size() > 0) {
      point lp = p->last_position();
      x = parse_option_coordinate('X', i, s, lp.x);
      y = parse_option_coordinate('Y', i, s, lp.y);
      z = parse_option_coordinate('Z', i, s, lp.z);
    } else {
      x = parse_coordinate('X', i, s);
      y = parse_coordinate('Y', i, s);
      z = parse_coordinate('Z', i, s);
    }
    return c.mk_G0(x, y, z);
  }
  
  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    string::size_type i = 0;
    while (i < s.size()) {
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
	  instr* is = parse_G0(c, p, &i, s);
	  p->push_back(is); //c.mk_G0(x, y, z));
	} else if (val == 1) {
	  instr* is = parse_G1(c, p, &i, s);
	  p->push_back(is);
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

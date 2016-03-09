#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include "core/parser.h"

namespace gca {

  template<typename T>
  struct parse_stream {
    size_t i;
    vector<T> s;

    template<typename R>
    parse_stream<T>(R v) : s(v.begin(), v.end()) {
      i = 0;
    }

    char next() {
      return s[i];
    }

    int chars_left() const {
      return i < s.size();
    }

    parse_stream<T>& operator++(int) {
      i++;
      return *this;
    }

    parse_stream<T>& operator--(int) {
      i--;
      return *this;
    }

    typename vector<T>::iterator end() {
      return s.end();
    }

    typename vector<T>::iterator begin() {
      return s.end();
    }
    
    typename vector<T>::iterator remaining() {
      return s.begin() + i;
    }

  };

  typedef parse_stream<char> parse_state;

  string string_remaining(parse_state& ps) {
    return string(ps.remaining(), ps.end());
  }

  void parse_char(char c, parse_state& s) {
    if (s.next() == c) {
      s++;
      return;
    }
    cout << "Cannot parse char " << c << " from string " << string_remaining(s) << endl;
    assert(false);
  }
  
  comment* parse_comment_with_delimiters(char sc, char ec, parse_state& s) {
    string text = "";
    parse_char(sc, s);
    while (s.next() != ec) {
      text += s.next();
      s++;
    }
    parse_char(ec, s);
    return comment::make(sc, ec, text);
  }

  void ignore_whitespace(parse_state& s) {
    while (s.chars_left() && isspace(s.next())) { s++; }
  }

  double parse_double(parse_state& s) {
    size_t j = s.i;
    double v =stod(string_remaining(s), &j);
    s.i += j;
    return v;
  }

  int parse_int(parse_state& s) {
    size_t j = s.i;
    int v = stoi(string_remaining(s), &j);
    s.i += j;
    return v;
  }

  double parse_coordinate(char c, parse_state& s) {
    ignore_whitespace(s);
    assert(s.next() == c);
    s++;
    return parse_double(s);
  }

  double parse_option_coordinate(char c, parse_state& s, double def=0.0) {
    ignore_whitespace(s);
    if (s.next() == c) {
      s++;
      return parse_double(s);
    }
    return def;
  }

  value* parse_option_value(char v, parse_state& s) {
    ignore_whitespace(s);
    if (s.next() == v) {
      parse_char(v, s);
      if (s.next() == '#') {
	parse_char('#', s);
	int val = parse_int(s);
	return var::make(val);
      } else {
	double d = parse_double(s);
	return lit::make(d);
      }
    }
    return omitted::make();
  }
  
  instr* parse_G1(gprog* p, parse_state& s) {
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', s);
    value* x = parse_option_value('X', s);
    value* y = parse_option_value('Y', s);
    value* z = parse_option_value('Z', s);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', s);
    }
    return g1_instr::make(x, y, z, fr);
  }

  instr* parse_G0(gprog* p, parse_state& s) {
    ignore_whitespace(s);
    value* x = parse_option_value('X', s);
    value* y = parse_option_value('Y', s);
    value* z = parse_option_value('Z', s);
    return g0_instr::make(x, y, z);
  }

  instr* parse_G2(gprog* p, parse_state& s) {
    ignore_whitespace(s);
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', s);
    value* x = parse_option_value('X', s);
    value* y = parse_option_value('Y', s);
    value* z = parse_option_value('Z', s);
    value* iv = parse_option_value('I', s);
    value* jv = parse_option_value('J', s);
    value* kv = parse_option_value('K', s);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', s);
    }    
    return g2_instr::make(x, y, z, iv, jv, kv, fr);
  }

  instr* parse_G3(gprog* p, parse_state& s) {
    ignore_whitespace(s);
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', s);    

    value* x = parse_option_value('X', s);
    value* y = parse_option_value('Y', s);
    value* z = parse_option_value('Z', s);
    
    value* iv = parse_option_value('I', s);
    value* jv = parse_option_value('J', s);
    value* kv = parse_option_value('K', s);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', s);
    }    
    return g3_instr::make(x, y, z, iv, jv, kv, fr);
  }
  
  instr* parse_G53(gprog* p, parse_state& s) {
    value* x = parse_option_value('X', s);
    value* y = parse_option_value('Y', s);
    value* z = parse_option_value('Z', s);
    return g53_instr::make(x, y, z);
  }

  instr* parse_G64(gprog* p, parse_state& s) {
    value* pv = parse_option_value('P', s);
    return g64_instr::make(pv);
  }

  instr* parse_G43(gprog* p, parse_state& s) {
    value* pv = parse_option_value('H', s);
    return g43_instr::make(pv);
  }
  
  string parse_option_char(parse_state& s, char t) {
    if (s.next() == t) {
      s++;
      char str[2];
      str[0] = t;
      str[1] = '\0';
      string c(str);
      return c;
    }
    return "";
  }

  string parse_coord_letters(parse_state& s) {
    string x = parse_option_char(s, 'X');
    string y = parse_option_char(s, 'Y');
    string z = parse_option_char(s, 'Z');
    return x + y + z;
  }

  instr* parse_ginstr(gprog* p,
		      int val,
		      parse_state& s) {
    instr* is;
    if (val == 0) {
      is = parse_G0(p, s);
    } else if (val == 1) {
      is = parse_G1(p, s);
    } else if (val == 91) {
      is = g91_instr::make();
    } else if (val == 90) {
      is = g90_instr::make();
    } else if (val == 53) {
      is = parse_G53(p, s);
    } else if (val == 2) {
      is = parse_G2(p, s);
    } else if (val == 3) {
      is = parse_G3(p, s);
    } else if (val == 21) {
      is = g21_instr::make();
    } else if (val == 64) {
      is = parse_G64(p, s);
    } else if (val == 43) {
      is = parse_G43(p, s);
    } else if (val == 17) {
      is = g17_instr::make();
    } else if (val == 18) {
      is = g18_instr::make();
    } else if (val == 19) {
      is = g19_instr::make();
    } else {
      cout << "Unrecognized instr code for instr letter: " << val << endl;
      assert(false);
    }
    ignore_whitespace(s);
    return is;
  }

  instr* parse_m_instr(int val, gprog* p, parse_state& s) {
    instr* is;
    if (val == 2) {
      is = m2_instr::make();
    } else if (val == 30) {
      is = m30_instr::make();
    } else if (val == 5) {
      is = m5_instr::make();
    } else if (val == 3) {
      is = m3_instr::make();
    } else if (val == 4) {
      is = m4_instr::make();
    } else if (val == 7) {
      is = m7_instr::make();
    } else if (val == 8) {
      is = m8_instr::make();
    } else if (val == 9) {
      is = m9_instr::make();
    } else {
      cout << "M value not supported " << val << endl;
      assert(false);
    }
    return is;
  }

  instr* parse_next_instr(int* g_register,
			  gprog* p,
			  parse_state& s) {
    instr* is;
    char next = s.next();
    if (next == '(') {
      is = parse_comment_with_delimiters('(', ')', s);
      return is;
    } else if (next == '[') {
      is = parse_comment_with_delimiters('[', ']', s);
      return is;
    }
    s++;
    if (next == 'M') {
      int val = parse_int(s);
      return parse_m_instr(val, p, s);
    } else if (next == 'T') {
      int val = parse_int(s);
      is = t_instr::make(val);
    } else if (next == 'S') {
      int val = parse_int(s);
      is = s_instr::make(val);
    } else if (next == 'F') {
      double val = parse_double(s);
      ignore_whitespace(s);
      string str = parse_coord_letters(s);
      is = f_instr::make(val, str);
    } else if (next == 'G') {
      int val = parse_int(s);
      is = parse_ginstr(p, val, s);
      if (val == 0 || val == 1 || val == 2 || val == 3) { *g_register = val; }
    } else if (next == '#') {
      int val = parse_int(s);
      parse_char('=', s);
      double e = parse_double(s);
      is = assign_instr::make(var::make(val), lit::make(e));
    } else if (next == 'Z' || next == 'X' || next == 'Y') {
      s--;
      assert(g_register != NULL);
      is = parse_ginstr(p, *g_register, s);
    } else {
      cout << "Cannot parse string: " << string_remaining(s) << endl;
      assert(false);
    }
    return is;
  }

  gprog* parse_gprog_line(int* g_register, parse_state& s) {
    gprog* p = gprog::make();
    while (s.chars_left()) {
      ignore_whitespace(s);
      if (!s.chars_left()) { break; }
      instr* is = parse_next_instr(g_register, p, s);
      p->push_back(is);
    }
    return p;
  }

  gprog* parse_gprog(const string& s) {
    gprog* p = gprog::make();
    stringstream ss(s);
    string ln;
    int* g_register = new int;
    while (getline(ss, ln, '\n')) {
      parse_state ps(ln);
      gprog* r = parse_gprog_line(g_register, ps);
      for (ilist::iterator it = r->begin(); it != r->end(); ++it) {
	p->push_back(*it);
      }
    }
    delete g_register;
    return p;
  }

  gprog* read_file(string file_name) {
    ifstream t(file_name);
    string str((istreambuf_iterator<char>(t)),
	       istreambuf_iterator<char>());
    return parse_gprog(str);
  }

}

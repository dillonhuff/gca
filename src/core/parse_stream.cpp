#include "core/parse_stream.h"

namespace gca {
  
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
  
  string parse_comment_with_delimiters(char sc, char ec, parse_state& s) {
    string text = "";
    parse_char(sc, s);
    while (s.next() != ec) {
      text += s.next();
      s++;
    }
    parse_char(ec, s);
    return text;
  }

  void ignore_whitespace(parse_state& s) {
    while (s.chars_left() && (isspace(s.next()) || s.next() == '%')) { s++; }
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

  double parse_option_coordinate(char c, parse_state& s, double def) {
    ignore_whitespace(s);
    if (s.next() == c) {
      s++;
      return parse_double(s);
    }
    return def;
  }

  value* parse_option_value(char v, parse_state& s) {
    ignore_whitespace(s);
    if (s.chars_left() == 0) {
      return omitted::make();
    }
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

}

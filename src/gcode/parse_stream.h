#ifndef GCA_PARSE_STREAM_H
#define GCA_PARSE_STREAM_H

#include <string>

#include "gcode/value.h"

using namespace std;

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

  string string_remaining(parse_state& ps);
  void parse_char(char c, parse_state& s);  
  string parse_comment_with_delimiters(char sc, char ec, parse_state& s);
  void ignore_whitespace(parse_state& s);
  double parse_double(parse_state& s);
  int parse_int(parse_state& s);
  double parse_coordinate(char c, parse_state& s);
  double parse_option_coordinate(char c, parse_state& s, double def=0.0);
  value* parse_option_value(char v, parse_state& s);  
}

#endif

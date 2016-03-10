#ifndef GCA_LEXER_H
#define GCA_LEXER_H

#include <vector>

#include "core/value.h"

using namespace std;

namespace gca {

  enum token_type {
    NEWLINE = 0,
    COMMENT,
    ICODE
  };

  struct token {
    virtual bool operator==(const token& other) const = 0;
    virtual token_type tp() const = 0;
  };
  
  struct newline : public token {
    virtual bool operator==(const token& other) const
    { return other.tp() == NEWLINE; }
    virtual token_type tp() const { return NEWLINE; }
  };

  struct cmt : public token {
    string text;
    
    virtual bool operator==(const token& other) const {
      if (other.tp() != NEWLINE)
	{return false; }
      const cmt& ci = static_cast<const cmt&>(other);
      return text == ci.text;
    }
    virtual token_type tp() const { return COMMENT; }
  };
  
  struct icode : public token {
    char c;
    const value& v;

    icode(char cp, const value& vp) : c(cp), v(vp) {}

    static icode* make(char c, const value& v) {
      return new (allocate<icode>()) icode(c, v);
    }
    
    virtual bool operator==(const token& other) const {
      if (other.tp() != ICODE)
	{return false; }
      const icode& ci = static_cast<const icode&>(other);
      return c == ci.c && v == ci.v;
    }
    virtual token_type tp() const { return ICODE; }
  };

  typedef vector<token*> block;

  ostream& operator<<(ostream& stream, const icode& ic);

  vector<block> lex_gprog(const string& s);

  bool operator==(const vector<block>& l, const vector<block>& r);
  
}

#endif

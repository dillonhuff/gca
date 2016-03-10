#ifndef GCA_LEXER_H
#define GCA_LEXER_H

#include <vector>

#include "core/value.h"

using namespace std;

namespace gca {

  struct token {};
  struct newline : public token {};
  struct cmt : public token {};
  struct icode : public token {
    char c;
    const value& v;
    icode(char cp, const value& vp) : c(cp), v(vp) {}
  };

  ostream& operator<<(ostream& stream, const icode& ic);

  vector<token> lex_gprog(const string& s);
  
}

#endif

#ifndef GCA_LEXER_H
#define GCA_LEXER_H

#include <vector>

using namespace std;

namespace gca {

  struct token {

  };

  vector<token> lex_gprog(const string& s);
  
}

#endif

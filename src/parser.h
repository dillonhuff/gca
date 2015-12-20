#ifndef GCA_PARSER_H
#define GCA_PARSER_H

#include <string>

#include "context.h"
#include "gprog.h"

using namespace std;

namespace gca {

  gprog* parse_gprog(context& c, string s);
  gprog* read_file(context& c, string file_name);
  
}

#endif

#include "bounds_checker.h"
#include "parser.h"

using namespace gca;
using namespace std;

int main() {
  string s = "/Users/dillon/CppWorkspace/gca/test/test_1.txt";
  context c;
  gprog* p = read_file(c, s);
  bounds_checker b(0, 9, -20, -10, 0.0, 2.0);
  b.check(cout, p);
  return 0;
}

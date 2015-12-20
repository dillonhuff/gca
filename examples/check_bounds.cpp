#include "bounds_checker.h"
#include "parser.h"

using namespace gca;
using namespace std;

// Test file:
// /Users/dillon/CppWorkspace/gca/test/test_1.txt

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "Usage is ./check-bounds-example <GCODE file path>" << endl;
    return 0;
  }
  string s(argv[1]);
  context c;
  gprog* p = read_file(c, s);
  bounds_checker b(0, 9, -20, -10, 0.0, 2.0);
  b.check(cout, p);
  return 0;
}

#include "bounds_checker.h"
#include "parser.h"

using namespace gca;
using namespace std;

/* 

   Bounds checker for GCODE programs with absolute coordinates.

   Test file: /Users/dillon/CppWorkspace/gca/test/test_1.txt

   To build the program
   ( from top level directory, e.g. /Users/dillon/CppWorkspace/gca/ )
   and run it on test file:
   cd /Users/dillon/CppWorkspace/gca/
   make clean
   make check-bounds-example -j
   ./check-bounds-example /Users/dillon/CppWorkspace/gca/test/test_1.txt

 */

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "Usage is ./check-bounds-example <GCODE file path>" << endl;
    return 0;
  }
  string s(argv[1]);
  
  gprog* p = read_file(s);
  // Adjust these to whatever boudns you would like to check
  int x_min = 0;
  int x_max = 9;
  int y_min = -20;
  int y_max = -10;
  int z_min = 0.0;
  int z_max = 2.0;
  bounds_checker b(x_min, x_max, y_min, y_max, z_min, z_max);
  b.check(cout, p);
  return 0;
}

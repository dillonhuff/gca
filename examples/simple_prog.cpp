#include "core/context.h"
#include "g0_filter.h"
#include "line.h"
#include "output.h"
#include "tiler.h"

using namespace gca;

int main() {
  double depth = -1;
  double l = 3;
  vector<line> square_edges;
  square_edges.push_back(line(point(0, 0, depth), point(0, l, depth)));
  square_edges.push_back(line(point(0, l, depth), point(l, l, depth)));
  square_edges.push_back(line(point(l, l, depth), point(l, 0, depth)));
  square_edges.push_back(line(point(l, 0, depth), point(0, 0, depth)));
  
  double cutter_width = 0.32;
  //double x_t = l + cutter_width;
  vector<cut*> square_cuts = lines_to_cuts(c, square_edges, cutter_width);
  gprog* square_prog = gcode_for_cuts(c, square_cuts);
  // tiler ht(2, point(0, 0, 0), point(x_t, x_t, 0));
  // square_prog = ht.apply(c, square_prog);
  // tiler vt(4, point(0, 0, 0), point(x_t*2, 0, 0));
  // square_prog = vt.apply(c, square_prog);
  g0_filter gf;
  square_prog = gf.apply(c, square_prog);
  cout << *square_prog;
  return 0;
}

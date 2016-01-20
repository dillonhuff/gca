#include <cstdlib>

#include "core/parser.h"
#include "simulators/sim_mill.h"
#include "system/bmp_output.h"

using namespace gca;

int main() {
  context c;
  gprog* p = parse_gprog(c, "G0 X0 Y0 Z2 G91 G1 X3 G0 X-3.0 M2");
  region r(5, 5, 5, 0.01);
  r.set_height(2, 3, 2, 4, 5);
  r.set_machine_x_offset(1);
  r.set_machine_y_offset(3);
  double tool_diameter = 1.0;
  cylindrical_bit t(tool_diameter);
  simulate_mill(*p, r, t);

  int w = r.num_x_elems;
  int h = r.num_y_elems;
  
  unsigned char* red = static_cast<unsigned char*>(malloc(sizeof(unsigned char)*w*h));
  unsigned char* green = static_cast<unsigned char*>(malloc(sizeof(unsigned char)*w*h));
  unsigned char* blue = static_cast<unsigned char*>(malloc(sizeof(unsigned char)*w*h));
  int i, j;
  for (i = 0; i < w; i++) {
    for (j = 0; j < w; j++) {
      //      cout << "Col height: (" << i << ", " << j << ") = " << r.column_height(i, j) << endl;
      int val = static_cast<int>(r.column_height(i, j));
      if (val == 5) {
	red[i*w + j] = 255;
      } else {
	red[i*w + j] = 0;
      }
      green[i*w + j] = 0;
      blue[i*w + j] = 0;
    }
  }

  write_bmp("sim_region", w, h, red, green, blue);

  free(red);
  free(green);
  free(blue);
}

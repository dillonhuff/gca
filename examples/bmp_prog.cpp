#include <cstdlib>

#include "core/parser.h"
#include "simulators/sim_mill.h"
#include "system/bmp_output.h"

using namespace gca;

int main() {
  
  gprog* p = parse_gprog("G0 X0 Y0 Z0 G91 G1 X3 Z4");
  region r(5, 5, 5, 0.01);
  r.set_height(0, 5, 0, 5, 5);
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
  double max = 0.0;
  int i, j;
  for (i = 0; i < w; i++) {
    for (j = 0; j < h; j++) {
      double c = r.column_height(i, j);
      if (c > max) {
	max = c;
      }
    }
  }

  cout << "Max = " << max << endl;
  //double red_pnt = 0.0;
  //double blue_pnt = max*(1.0/3.0);
  //double green_pnt = max*(2.0/3.0);
  for (i = 0; i < w; i++) {
    for (j = 0; j < h; j++) {
      //int h = static_cast<int>(r.column_height(i, j)/max)*100;
      blue[i*w + j] = 1; //(255*h)/100; //h > red_pnt ? static_cast<int>((h / max)*255.0) : 0;
      green[i*w + j] = 1; //(255*(100 - h))/100; //h > blue_pnt ? static_cast<int>((h / max)*255.0) : 0;
      red[i*w + j] = 0; //h > green_pnt ? static_cast<int>((h / max)*255.0) : 0;
    }
  }

  write_bmp("sim_region", w, h, red, green, blue);

  free(red);
  free(green);
  free(blue);
}

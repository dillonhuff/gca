#ifndef GCA_BMP_OUTPUT_H
#define GCA_BMP_OUTPUT_H

#include <string>

using namespace std;

namespace gca {

  void write_bmp(string output_file_name,
		 int w, int h,
		 unsigned char* red,
		 unsigned char* green,
		 unsigned char* blue);

}

#endif

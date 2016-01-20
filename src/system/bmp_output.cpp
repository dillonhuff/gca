#include <cstdio>
#include <cstdlib>

#include "system/bmp_output.h"

namespace gca {

  void write_bmp(string output_file_name,
		 int w, int h,
		 unsigned char* red,
		 unsigned char* green,
		 unsigned char* blue) {
    int x, y, r, g, b;
    int i, j;
    FILE* f;
    unsigned char* img = static_cast<unsigned char*>(malloc(3*w*h));
    int filesize = 54 + 3*w*h;
    memset(img, 0, 3*w*h);

    for(i=0; i<w; i++)
      {
	for(j=0; j<h; j++)
	  {
	    x=i; y=(h-1)-j;
	    r = red[i*w + j]*255;
	    g = green[i*w + j]*255;
	    b = blue[i*w + j]*255;
	    if (r > 255) r=255;
	    if (g > 255) g=255;
	    if (b > 255) b=255;
	    img[(x+y*w)*3+2] = (unsigned char)(r);
	    img[(x+y*w)*3+1] = (unsigned char)(g);
	    img[(x+y*w)*3+0] = (unsigned char)(b);
	  }
      }

    unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
    unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
    unsigned char bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (unsigned char)(filesize    );
    bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
    bmpfileheader[ 4] = (unsigned char)(filesize>>16);
    bmpfileheader[ 5] = (unsigned char)(filesize>>24);

    bmpinfoheader[ 4] = (unsigned char)(       w    );
    bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
    bmpinfoheader[ 6] = (unsigned char)(       w>>16);
    bmpinfoheader[ 7] = (unsigned char)(       w>>24);
    bmpinfoheader[ 8] = (unsigned char)(       h    );
    bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
    bmpinfoheader[10] = (unsigned char)(       h>>16);
    bmpinfoheader[11] = (unsigned char)(       h>>24);

    f = fopen((output_file_name + ".bmp").c_str(), "wb");
    fwrite(bmpfileheader,1,14,f);
    fwrite(bmpinfoheader,1,40,f);
    for(i=0; i<h; i++)
      {
	fwrite(img+(w*(h-i-1)*3),3,w,f);
	fwrite(bmppad,1,(4-(w*3)%4)%4,f);
      }
    fclose(f);
  }
}

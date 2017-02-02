#ifndef GCA_DXF_READER_H
#define GCA_DXF_READER_H

#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"
#include "gcode/linear_cut.h"
#include "backend/shape_layout.h"

namespace gca {

  shape_layout read_dxf(const char* file, bool log=false);

}

#endif

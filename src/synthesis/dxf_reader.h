#ifndef GCA_DXF_READER_H
#define GCA_DXF_READER_H

#include "core/gprog.h"
#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shape_layout.h"

namespace gca {

  shape_layout read_dxf(const char* file, bool log=false);

}

#endif

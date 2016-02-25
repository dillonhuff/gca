#ifndef GCA_SHAPES_TO_TOOLPATHS_H
#define GCA_SHAPES_TO_TOOLPATHS_H

#include "synthesis/shape_layout.h"
#include "synthesis/toolpath.h"

namespace gca {

  vector<toolpath> cut_toolpaths(const shape_layout& shapes,
				 const cut_params& params);
  
  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params);
  
}

#endif

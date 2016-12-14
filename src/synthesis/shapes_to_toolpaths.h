#ifndef GCA_SHAPES_TO_TOOLPATHS_H
#define GCA_SHAPES_TO_TOOLPATHS_H

#include "geometry/polyline.h"
#include "synthesis/shape_layout.h"
#include "backend/toolpath.h"

namespace gca {

  vector<cut*> cuts_from_polylines(const shape_layout& shapes_to_cut,
				   const vector<polyline>& ps,
				   const cut_params& params);
  
  vector<polyline> polylines_for_shapes(const shape_layout& shapes_to_cut);

  // TODO: The fact that shapes_to_cut must be passed to cuts_from_polylines
  // is an abomination
  template<typename T>
  vector<cut*> shape_cuts_p(const shape_layout& shapes_to_cut,
			    const cut_params& params,
			    T t) {
    vector<polyline> ps = t(polylines_for_shapes(shapes_to_cut));
    return cuts_from_polylines(shapes_to_cut, ps, params);
  }
  
  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params);
  
}

#endif

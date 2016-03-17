#include <algorithm>
#include <numeric>

#include "geometry/polyline.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"

namespace gca {

  vector<cut*> hole_cuts(const vector<hole_punch*>& holes,
  			 const cut_params& params) {
    vector<cut*> cuts;
    for (vector<hole_punch*>::const_iterator it = holes.begin();
  	 it != holes.end(); ++it) {
      cuts.push_back(*it);
    }
    return cuts;
  }
  
  tool_name select_tool(ToolOptions tools) {
    if (tools == DRILL_AND_DRAG_KNIFE ||
    	tools == DRAG_KNIFE_ONLY) {
      return DRAG_KNIFE;
    } else if (tools == DRILL_ONLY) {
      return DRILL;
    } else {
      assert(false);
    }
  }

  vector<double> cut_depths(const cut_params& params) {
    vector<double> depths;
    double depth = max(0.0, params.material_depth - params.cut_depth);
    while (true) {
      depths.push_back(depth);
      if (depth == 0.0) {
	break;
      }
      depth = max(0.0, depth - params.cut_depth);
    }
    return depths;
  }

  void set_tool_nos(tool_name tool_no,
		    const vector<cut*>& cuts) {
    for (vector<cut*>::const_iterator it = cuts.begin(); it != cuts.end(); ++it) {
      cut* c = *it;
      if (c->is_hole_punch()) {
	c->tool_no = DRILL;
      } else {
	c->tool_no = tool_no;
      }
    }
  }

  void append_deepened_cuts(cut* ct,
			    vector<cut*>& cuts,
			    const vector<double>& depths,
			    const cut_params& params) {
    for (vector<double>::const_iterator it = depths.begin();
	 it != depths.end(); ++it) {
      double depth = *it;
      cut* cut_with_depth;
      assert(ct->start.z == 0.0 && ct->end.z == 0.0);
      if (ct->is_linear_cut()) {
	cut_with_depth = linear_cut::make(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth));
      } else if (ct->is_circular_arc()) {
	cut_with_depth = ct->copy();
	cut_with_depth->start.z = depth;
	cut_with_depth->end.z = depth;
      } else {
	assert(false);
      }
      cuts.push_back(cut_with_depth);      
    }
  }

  void insert_lines(const vector<cut*>& lines,
		    vector<polyline>& polys) {
    for (auto c : lines) {
      vector<point> p{c->start, c->end};
      polys.push_back(polyline(p));
    }
  }
  
  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params) {
    vector<polyline> polys;
    //append_splines(shapes_to_cut.splines, no_depth_cuts);
    insert_lines(shapes_to_cut.lines, polys);
    vector<cut*> no_depth_cuts;
    for (auto pl : polys) {
      for (auto l : pl.lines()) {
	no_depth_cuts.push_back(linear_cut::make(l.start, l.end));
      }
    }
    vector<double> depths = cut_depths(params);
    vector<cut*> cuts;
    if (params.tools != DRAG_KNIFE_ONLY) {
      vector<cut*> dcs = hole_cuts(shapes_to_cut.holes, params);
      cuts.insert(cuts.end(), dcs.begin(), dcs.end());
    }
    for (vector<cut*>::iterator it = no_depth_cuts.begin();
    	 it != no_depth_cuts.end(); ++it) {
      append_deepened_cuts(*it, cuts, depths, params);
    }
    tool_name tool_no = select_tool(params.tools);
    set_tool_nos(tool_no, cuts);
    return cuts;
  }
  
}

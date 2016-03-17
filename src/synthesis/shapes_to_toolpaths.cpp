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

  void insert_lines(const vector<cut*>& lines,
		    vector<polyline>& polys) {
    for (auto c : lines) {
      vector<point> p{c->start, c->end};
      polys.push_back(polyline(p));
    }
  }

  vector<polyline> deepen_polyline(vector<double> depths, const polyline& p) {
    vector<polyline> ps;
    for (auto depth : depths) {
      vector<point> pts;
      for (auto pt : p) {
	pts.push_back(point(pt.x, pt.y, depth));
      }
      ps.push_back(polyline(pts));
    }
    return ps;
  }
  
  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params) {
    vector<polyline> polys;
    append_splines(shapes_to_cut.splines, polys);
    insert_lines(shapes_to_cut.lines, polys);
    vector<double> depths = cut_depths(params);
    vector<cut*> cuts;
    for (auto pl : polys) {
      for (auto dpl : deepen_polyline(depths, pl)) {
	for (auto l : dpl.lines())
	  { cuts.push_back(linear_cut::make(l.start, l.end)); }
      }
    }
    if (params.tools != DRAG_KNIFE_ONLY) {
      vector<cut*> dcs = hole_cuts(shapes_to_cut.holes, params);
      cuts.insert(cuts.end(), dcs.begin(), dcs.end());
    }
    tool_name tool_no = select_tool(params.tools);
    set_tool_nos(tool_no, cuts);
    return cuts;
  }
  
}

#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/circular_arc.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"

namespace gca {

  void make_pass(int tool_number,
		 double depth,
		 const cut_params& params,
		 const vector<cut*>& current_group,
		 vector<cut_group>& cut_group_passes) {
    vector<cut*> new_pass;
    for (unsigned i = 0; i < current_group.size(); i++) {
      cut* ct = current_group[i];
      assert(ct->start.z == 0.0 && ct->end.z == 0.0);
      if (ct->is_linear_cut()) {
	linear_cut* lct = linear_cut::make(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth));
	lct->tool_no = tool_number;
	new_pass.push_back(lct);
      } else if (ct->is_circular_arc()) {
	circular_arc* arc = static_cast<circular_arc*>(ct);
	point s = arc->start;
	s.z = params.pass_depth;
	point e = arc->end;
	e.z = params.pass_depth;
	circular_arc* ct = circular_arc::make(s, e, arc->start_offset, arc->dir, arc->pl);
	ct->tool_no = tool_number;
	new_pass.push_back(ct);
      } else {
	assert(false);
      }
    }
    cut_group_passes.push_back(new_pass);
  }		 

  void make_cut_group_passes(int tool_number,
			     const cut_params& params,
			     const vector<double>& depths,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
    for (vector<double>::const_iterator it = depths.begin();
	 it != depths.end(); ++it) {
      make_pass(tool_number,
		*it,
		params,
		current_group,
		cut_group_passes);	
    }
  }
  
  vector<cut*> hole_cuts(const vector<hole_punch*>& holes,
		     cut_params params) {
    vector<cut*> cuts;
    for (unsigned i = 0; i < holes.size(); i++) {
      if (params.one_pass_only) {
    	hole_punch* h = holes[i];
    	hole_punch* nh = hole_punch::make(point(h->start.x, h->start.y, params.pass_depth), h->radius);
    	nh->tool_no = 2;
    	cuts.push_back(nh);
      } else {
	hole_punch* h = holes[i];
    	hole_punch* nh = hole_punch::make(h->start, h->radius);
    	nh->tool_no = 2;
    	cuts.push_back(nh);
      }
    }
    return cuts;
  }

  vector<cut*> line_cuts(int tool_number,
			 const cut_group& current_group,
			 const vector<double> depths,
			 cut_params params) {
    vector<cut_group> cut_group_passes;
    make_cut_group_passes(tool_number,
			  params,
			  depths,
			  current_group,
			  cut_group_passes);
    vector<cut*> cuts;
    for (vector<cut_group>::iterator it = cut_group_passes.begin();
	 it != cut_group_passes.end(); ++it) {
      cuts.insert(cuts.end(), (*it).begin(), (*it).end());
    }
    return cuts;
  }

  int select_tool(ToolOptions tools) {
    if (tools == DRILL_AND_DRAG_KNIFE ||
    	tools == DRAG_KNIFE_ONLY) {
      return 6;
    } else if (tools == DRILL_ONLY) {
      return 2;
    } else {
      assert(false);
    }
  }

  vector<double> cut_depths(const cut_params& params) {
    vector<double> depths;
    if  (params.one_pass_only) {
      depths.push_back(params.pass_depth);
    } else {
      assert(params.cut_depth < params.material_depth);
      double depth = params.material_depth - params.cut_depth;
      while (true) {
	depths.push_back(depth);
	if (depth == 0.0) {
	  break;
	}
	depth = max(0.0, depth - params.cut_depth);
      }
    }
    return depths;
  }

  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params) {
    vector<cut*> cuts;
    if (params.tools != DRAG_KNIFE_ONLY) {
      vector<cut*> dcs = hole_cuts(shapes_to_cut.holes, params);
      cuts.insert(cuts.end(), dcs.begin(), dcs.end());
    }
    vector<cut_group> cut_groups;
    append_splines(shapes_to_cut.splines, cut_groups);
    group_adjacent_cuts(shapes_to_cut.lines, cut_groups, 30.0);

    int tool_no = select_tool(params.tools);
    vector<double> depths = cut_depths(params);
    for (vector<cut_group>::iterator it = cut_groups.begin();
    	 it != cut_groups.end(); ++it) {
      vector<cut*> ct = line_cuts(tool_no, *it, depths, params);
      cuts.insert(cuts.end(), ct.begin(), ct.end());
    }
    
    return cuts;
  }
  
}

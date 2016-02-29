#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/circular_arc.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"

namespace gca {

  void make_pass(int tool_number,
		 double depth,
		 const vector<cut*>& current_group,
		 vector<cut_group>& cut_group_passes) {
    vector<cut*> new_pass;
    for (unsigned i = 0; i < current_group.size(); i++) {
      cut* ct = current_group[i];
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
      new_pass.push_back(cut_with_depth);
    }
    cut_group_passes.push_back(new_pass);
  }		 

  void make_cut_group_passes(int tool_number,
			     const vector<double>& depths,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
    for (vector<double>::const_iterator it = depths.begin();
	 it != depths.end(); ++it) {
      make_pass(tool_number,
		*it,
		current_group,
		cut_group_passes);	
    }
  }
  
  vector<cut*> hole_cuts(const vector<hole_punch*>& holes,
			 const cut_params& params) {
    vector<cut*> cuts;
    for (unsigned i = 0; i < holes.size(); i++) {
      cuts.push_back(holes[i]);
    }
    return cuts;
  }

  vector<cut*> line_cuts(int tool_number,
			 const cut_group& current_group,
			 const vector<double> depths,
			 const cut_params& params) {
    vector<cut_group> cut_group_passes;
    make_cut_group_passes(tool_number,
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

    tool_name tool_no = select_tool(params.tools);
    vector<double> depths = cut_depths(params);
    for (vector<cut_group>::iterator it = cut_groups.begin();
    	 it != cut_groups.end(); ++it) {
      vector<cut*> ct = line_cuts(tool_no, *it, depths, params);
      cuts.insert(cuts.end(), ct.begin(), ct.end());
    }
    set_tool_nos(tool_no, cuts);
    return cuts;
  }

}

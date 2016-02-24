#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/circular_arc.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"

namespace gca {

  // TODO: This is horrible. Need to add more tests and pull it apart
  void make_cut_group_passes(cut_params params,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
    if  (params.one_pass_only) {
      vector<cut*> new_pass;
      for (unsigned i = 0; i < current_group.size(); i++) {
	cut* ct = current_group[i];
	if (ct->is_linear_cut()) {
	  new_pass.push_back(mk_linear_cut(point(ct->start.x, ct->start.y, params.pass_depth), point(ct->end.x, ct->end.y, params.pass_depth)));
	} else if (ct->is_circular_arc()) {
	  circular_arc* arc = static_cast<circular_arc*>(ct);
	  point s = arc->start;
	  s.z = params.pass_depth;
	  point e = arc->end;
	  e.z = params.pass_depth;
	  new_pass.push_back(circular_arc::make(s, e, arc->start_offset, arc->dir, arc->pl));
	} else {
	  assert(false);
	}
      }
      cut_group_passes.push_back(new_pass);
      
    } else {
      assert(params.cut_depth < params.material_depth);
      double depth = params.material_depth - params.cut_depth;
      while (true) {
	vector<cut*> new_pass;
	for (unsigned i = 0; i < current_group.size(); i++) {
	  cut* ct = current_group[i];
	  assert(ct->is_linear_cut());
	  new_pass.push_back(mk_linear_cut(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth)));
	}
	cut_group_passes.push_back(new_pass);
	if (depth == 0.0) {
	  break;
	}
	depth = max(0.0, depth - params.cut_depth);
      }
    }
  }
  
  toolpath drill_toolpath(const vector<hole_punch*>& holes,
			  cut_params params) {
    toolpath t;
    t.tool_no = 2;
    vector<cut_group> punch_groups;
    for (unsigned i = 0; i < holes.size(); i++) {
      cut_group one_hole;
      if (params.one_pass_only) {
	hole_punch* h = holes[i];
	hole_punch* nh = hole_punch::make(point(h->start.x, h->start.y, params.pass_depth), h->radius);
	nh->tool_no = h->tool_no;
	one_hole.push_back(nh);
      } else {
	one_hole.push_back(holes[i]);
      }
      punch_groups.push_back(one_hole);
    }
    t.cut_groups = punch_groups;
    return t;
  }

  toolpath cut_toolpath(int tool_number,
			const cut_group& current_group,
			cut_params params) {
    toolpath t;
    t.tool_no = tool_number;
    vector<cut_group> cut_group_passes;
    make_cut_group_passes(params,
			  current_group,
			  cut_group_passes);
    t.cut_groups = cut_group_passes;
    return t;
  }

  vector<toolpath> shape_toolpaths(const shape_layout& shapes_to_cut,
				 const cut_params& params) {
    vector<toolpath> toolpaths;
    if (params.tools != DRAG_KNIFE_ONLY) {
      toolpaths.push_back(drill_toolpath(shapes_to_cut.holes, params));
    }
    vector<cut_group> cut_groups;
    append_splines(shapes_to_cut.splines, cut_groups);
    group_adjacent_cuts(shapes_to_cut.lines, cut_groups, 30.0);

    int tool_no;
    if (params.tools == DRILL_AND_DRAG_KNIFE ||
	params.tools == DRAG_KNIFE_ONLY) {
      tool_no = 6;
    } else if (params.tools == DRILL_ONLY) {
      tool_no = 2;
    } else {
      assert(false);
    }
    for (vector<cut_group>::iterator it = cut_groups.begin();
	 it != cut_groups.end(); ++it) {
      toolpaths.push_back(cut_toolpath(tool_no, *it, params));
    }
    return toolpaths;
  }

  int cmp_tools(const toolpath& l, const toolpath& r) {
    return l.tool_no < r.tool_no;
  }

  bool toolpath_is_empty(const toolpath& t) {
    return t.cut_groups.size() == 0;
  }

  toolpath transition_toolpath(const toolpath& next, const toolpath& previous) {
    toolpath t;
    return t;
  }
  
  vector<toolpath> cut_toolpaths(const shape_layout& shapes_to_cut,
				 const cut_params& params) {
    vector<toolpath> toolpaths = shape_toolpaths(shapes_to_cut, params);
    toolpaths.erase(remove_if(toolpaths.begin(), toolpaths.end(), toolpath_is_empty),
    		    toolpaths.end());
    stable_sort(toolpaths.begin(), toolpaths.end(), cmp_tools);
    return toolpaths;
  }

}

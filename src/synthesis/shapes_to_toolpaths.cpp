#include <algorithm>
#include <numeric>

#include "core/context.h"
#include "synthesis/circular_arc.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"

namespace gca {

  // TODO: This is horrible. Need to add more tests and pull it apart
  void make_cut_group_passes(int tool_number,
			     cut_params params,
			     const vector<cut*>& current_group,
			     vector<cut_group>& cut_group_passes) {
    cout << "Current group size = " << current_group.size() << endl;
    if  (params.one_pass_only) {
      vector<cut*> new_pass;
      for (unsigned i = 0; i < current_group.size(); i++) {
      	cut* ct = current_group[i];
      	if (ct->is_linear_cut()) {
	  linear_cut* lct = linear_cut::make(point(ct->start.x, ct->start.y, params.pass_depth), point(ct->end.x, ct->end.y, params.pass_depth));
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
    } else {
      assert(params.cut_depth < params.material_depth);
      double depth = params.material_depth - params.cut_depth;
      while (true) {
	vector<cut*> new_pass;
	for (unsigned i = 0; i < current_group.size(); i++) {
	  cut* ct = current_group[i];
	  assert(ct->is_linear_cut());
	  linear_cut* lc = linear_cut::make(point(ct->start.x, ct->start.y, depth), point(ct->end.x, ct->end.y, depth));
	  lc->tool_no = tool_number;
	  new_pass.push_back(lc);
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
			 cut_params params) {
    vector<cut_group> cut_group_passes;
    make_cut_group_passes(tool_number,
			  params,
			  current_group,
			  cut_group_passes);
    vector<cut*> cuts;
    for (vector<cut_group>::iterator it = cut_group_passes.begin();
	 it != cut_group_passes.end(); ++it) {
      cuts.insert(cuts.end(), (*it).begin(), (*it).end());
    }
    return cuts;
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
      vector<cut*> ct = line_cuts(tool_no, *it, params);
      cuts.insert(cuts.end(), ct.begin(), ct.end());
    }
    
    return cuts;
  }
  
}

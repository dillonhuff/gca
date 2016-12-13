#include "process_planning/axis_location.h"
#include "process_planning/direction_selection.h"
#include "process_planning/feature_selection.h"

namespace gca {

  void clip_top_and_bottom_pairs(std::vector<direction_process_info>& dirs,
				 const std::vector<tool>& tools) {
    for (unsigned i = 0; i < dirs.size(); i++) {
      feature_decomposition* l = dirs[i].decomp;

      bool found_opposite = false;

      for (unsigned j = i + 1; j < dirs.size(); j++) {
	feature_decomposition* r = dirs[j].decomp;

	if (angle_eps(normal(l), normal(r), 180.0, 0.5)) {
	  found_opposite = true;

	  clip_leaves(l, r);
	  clip_leaves(r, l);

	  dirs[i].tool_info = find_accessable_tools(l, tools);
	  dirs[j].tool_info = find_accessable_tools(r, tools);

	  break;
	}

      }
    }

    for (unsigned i = 0; i < dirs.size(); i++) {
      auto decomp = dirs[i].decomp;
      auto& acc_info = dirs[i].tool_info;

      delete_leaves(decomp, [acc_info](feature* f) {
      	  return map_find(f, acc_info).size() == 0;
      	});

    }

    for (unsigned i = 0; i < dirs.size(); i++) {
      feature_decomposition* l = dirs[i].decomp;

      for (unsigned j = i + 1; j < dirs.size(); j++) {
	feature_decomposition* r = dirs[j].decomp;

	if (angle_eps(normal(l), normal(r), 180.0, 0.5)) {

	  clip_top_and_bottom_features(l, r);

	  break;
	}

      }
    }
    
  }

  void
  delete_inaccessable_non_leaf_nodes(feature_decomposition* decomp,
				     const tool_access_info& acc_info) {

    delete_internal_nodes(decomp, [acc_info](feature* f) {
	return map_find(f, acc_info).size() == 0;
      });
  }

  direction_process_info
  build_direction_info(const triangular_mesh& stock,
		       const triangular_mesh& part,
		       const point n,
		       const std::vector<tool>& tools) {
    feature_decomposition* decomp = build_feature_decomposition(stock, part, n);
    tool_access_info info = find_accessable_tools(decomp, tools);
    vector<chamfer> chamfers = chamfer_regions(part, n, tools);
    // vector<freeform_surface> freeform_surfs =
    //   freeform_surface_regions(part, n, tools);

    // cout << "# of freeform surfaces in " <<  n <<  " = " << freeform_surfs.size() << endl;

    return {decomp, info, chamfers, {}}; //freeform_surfs};
  }
  
  std::vector<direction_process_info>
  initial_decompositions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const std::vector<tool>& tools,
			 const std::vector<point>& norms) {

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      direction_process_info info =
	build_direction_info(stock, part, n, tools);
      dir_info.push_back(info);
    }

    for (auto info : dir_info) {
      feature_decomposition* decomp = info.decomp;

#ifdef VIZ_DBG
      vtk_debug_feature_decomposition(decomp);
#endif

      tool_access_info& acc_info = info.tool_info;

      delete_inaccessable_non_leaf_nodes(decomp, acc_info);

#ifdef VIZ_DBG
      vtk_debug_feature_decomposition(decomp);
#endif

    }

    clip_top_and_bottom_pairs(dir_info, tools);

    return dir_info;
  }

  std::vector<direction_process_info>
  select_mill_directions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const fixtures& f,
			 const std::vector<tool>& tools) {
    vector<point> norms = select_cut_directions(stock, part, f, tools);
    vector<direction_process_info> dir_info =
      initial_decompositions(stock, part, tools, norms);

    return dir_info;
  }

}

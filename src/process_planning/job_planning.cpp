#include <vtkDelaunay2D.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

#include "geometry/extrusion.h"
#include "geometry/mesh_operations.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "process_planning/feature_to_pocket.h"
#include "process_planning/job_planning.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/check.h"

namespace gca {

  fixture_setup
  create_setup(const homogeneous_transform& s_t,
	       const triangular_mesh& wp_mesh,
	       const triangular_mesh& part_mesh,
	       feature_decomposition* decomp,
	       const fixture& f,
	       const tool_access_info& tool_info) {
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    vector<pocket> pockets = feature_pockets(*decomp, s_t, tool_info);

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  triangular_mesh feature_mesh(const feature& f) {
    auto m = extrude(f.base(), (1.0001 + f.depth())*f.normal());

    DBG_ASSERT(m.is_connected());

    return m;
  }

  triangular_mesh subtract_features(const triangular_mesh& m,
				    feature_decomposition* features) {
    auto fs = collect_features(features);

    cout << "Got all features" << endl;

    std::vector<triangular_mesh> meshes;
    for (auto f : fs) {
      meshes.push_back(feature_mesh(*f));
    }

    cout << "Got all meshes" << endl;

    auto subtracted = boolean_difference(m, meshes);

    return subtracted;
  }

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
    double part_volume = volume(part);

    vector<surface> surfs = outer_surfaces(stock);

    DBG_ASSERT(surfs.size() == 6);

    vector<point> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
    }

    DBG_ASSERT(norms.size() == 6);

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      feature_decomposition* decomp = build_feature_decomposition(stock, part, n);
      tool_access_info info = find_accessable_tools(decomp, tools);
      dir_info.push_back({decomp, info});
    }

    vice v = f.get_vice();
    triangular_mesh current_stock = stock;
    vector<fixture_setup> cut_setups;

    vtk_debug_mesh(current_stock);

    for (auto& info : dir_info) {
      auto sfs = outer_surfaces(current_stock);
      auto orients = all_stable_orientations(sfs, v);
      point n = normal(info.decomp);
      auto maybe_orient =
	find_orientation_by_normal_optional(orients, n);

      if (maybe_orient) {

	auto orient = *maybe_orient;
	fixture fix(orient, v);

	auto decomp = info.decomp;
	auto& acc_info = info.tool_info;

	delete_nodes(decomp, [acc_info](feature* f) {
	    return map_find(f, acc_info).size() == 0;
	  });

	auto t = mating_transform(current_stock, orient, v);
	cut_setups.push_back(create_setup(t, current_stock, part, decomp, fix, info.tool_info));

	current_stock = subtract_features(current_stock, decomp);

	double stock_volume = volume(current_stock);
	double volume_ratio = part_volume / stock_volume;
	
	cout << "Part volume  = " << part_volume << endl;
	cout << "Stock volume = " << stock_volume << endl;
	cout << "part / stock = " << volume_ratio << endl;

	vtk_debug_mesh(current_stock);

	if (volume_ratio > 0.999) { break; }

      }
    }

    vtk_debug_mesh(current_stock);

    return cut_setups;
  }
  
}

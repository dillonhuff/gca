#include "geometry/vtk_utils.h"
#include "synthesis/visual_debug.h"

namespace gca {

  vtkSmartPointer<vtkPolyData>
  polydata_for_vice(const vice v) {
    box main = main_box(v);
    box upper_clamp = upper_clamp_box(v);
    box lower_clamp = lower_clamp_box(v);

    vector<triangle> triangles;
    concat(triangles, box_triangles(main));
    concat(triangles, box_triangles(upper_clamp));
    concat(triangles, box_triangles(lower_clamp));
    return polydata_for_triangles(triangles);
  }
  
  void visual_debug(const fabrication_setup& setup) {
    auto vice_pd = polydata_for_vice(setup.v);
    auto vice_actor = polydata_actor(vice_pd);

    vector<vtkSmartPointer<vtkActor>> actors{vice_actor};
    auto a = setup.arrangement();
    for (auto n : a.mesh_names()) {
      if (a.metadata(n).display_during_debugging) {
	auto other_pd = polydata_for_trimesh(a.mesh(n));
	actors.push_back(polydata_actor(other_pd));
      }
    }
    visualize_actors(actors);
  }
  
}

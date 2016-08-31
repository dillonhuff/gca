#include "feature_recognition/visual_debug.h"

namespace gca {

  void vtk_debug_polygon(const labeled_polygon_3& p) {
    vector<vtkSmartPointer<vtkPolyData>> ring_pds;

    auto pd = polydata_for_ring(p.vertices());
    ring_pds.push_back(pd);
    cout << "??? # lines in poly = " << pd->GetNumberOfPolys() << endl;
    
    for (auto ir : p.holes()) {
      auto pd = polydata_for_ring(ir);
      cout << "??? # lines in hole? = " << pd->GetNumberOfPolys() << endl;
      ring_pds.push_back(polydata_for_ring(ir));
    }
    
    vector<vtkSmartPointer<vtkActor>> ring_acts;
    for (auto r : ring_pds) {
      ring_acts.push_back(polydata_actor(r));
    }

    visualize_actors(ring_acts);
  }

  void vtk_debug_polygons(const std::vector<labeled_polygon_3>& polys) {
    vector<vtkSmartPointer<vtkPolyData>> ring_pds;

    for (auto p : polys) {
      auto pd = polydata_for_ring(p.vertices());
      ring_pds.push_back(pd);
    
      for (auto ir : p.holes()) {
	auto pd = polydata_for_ring(ir);
	ring_pds.push_back(polydata_for_ring(ir));
      }
    }
    
    vector<vtkSmartPointer<vtkActor>> ring_acts;
    for (auto r : ring_pds) {
      ring_acts.push_back(polydata_actor(r));
    }

    visualize_actors(ring_acts);
  }
  
  void vtk_debug_feature_decomposition(feature_decomposition* f) {
    vector<feature*> non_void_features;

    DBG_ASSERT(false);
  }

  void vtk_debug_feature_tree(feature_decomposition* f) {
    cout << "Traversing feature tree" << endl;

    auto display_feature_base = [](feature* f) {
      if (f != nullptr) {
    	vtk_debug_polygon(f->base());
      }
    };
    traverse_bf(f, display_feature_base);

    cout << "Done traversing feature tree" << endl;
  }
    

}

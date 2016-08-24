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

  void vtk_debug_feature_decomposition(feature_decomposition* f) {
    vector<feature*> non_void_features;

    DBG_ASSERT(false);
  }
}

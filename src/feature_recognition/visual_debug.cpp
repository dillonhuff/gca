#include "feature_recognition/visual_debug.h"

namespace gca {

  std::vector<vtkSmartPointer<vtkActor>>
  polygon_3_actors(const polygon_3& p) {
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

    return ring_acts;
  }
  
  void vtk_debug_polygon(const labeled_polygon_3& p) {
    // vector<vtkSmartPointer<vtkPolyData>> ring_pds;

    // auto pd = polydata_for_ring(p.vertices());
    // ring_pds.push_back(pd);
    // cout << "??? # lines in poly = " << pd->GetNumberOfPolys() << endl;
    
    // for (auto ir : p.holes()) {
    //   auto pd = polydata_for_ring(ir);
    //   cout << "??? # lines in hole? = " << pd->GetNumberOfPolys() << endl;
    //   ring_pds.push_back(polydata_for_ring(ir));
    // }
    
    vector<vtkSmartPointer<vtkActor>> ring_acts = polygon_3_actors(p);
    // for (auto r : ring_pds) {
    //   ring_acts.push_back(polydata_actor(r));
    // }

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
    vtk_debug_features(collect_features(f));
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

  vector<vtkSmartPointer<vtkActor>> feature_actors(const feature& f) {
    auto base_pd = f.base();
    auto top_pd = f.top();

    vector<vtkSmartPointer<vtkPolyData>> ring_pds;

    auto pd = polydata_for_ring(base_pd.vertices());
    ring_pds.push_back(pd);
    
    for (auto ir : base_pd.holes()) {
      auto pd = polydata_for_ring(ir);
      ring_pds.push_back(polydata_for_ring(ir));
    }

    auto bp = polydata_for_ring(top_pd.vertices());
    color_polydata(bp, 0, 200, 0);
    ring_pds.push_back(bp);
    
    for (auto ir : top_pd.holes()) {
      auto pd = polydata_for_ring(ir);
      color_polydata(pd, 0, 200, 0);
      ring_pds.push_back(pd);
    }
    
    vector<vtkSmartPointer<vtkActor>> ring_acts;
    for (auto r : ring_pds) {
      ring_acts.push_back(polydata_actor(r));
    }

    return ring_acts;
  }
  
  void vtk_debug_feature(const feature& f) {
    auto ring_acts = feature_actors(f);
    visualize_actors(ring_acts);
  }

  void vtk_debug_features(const std::vector<feature*>& fs) {
    vector<vtkSmartPointer<vtkActor>> actors;
    for (auto f : fs) {
      concat(actors, feature_actors(*f));
    }

    visualize_actors(actors);
  }

}

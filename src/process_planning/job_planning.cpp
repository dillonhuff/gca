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

    auto pockets = feature_pockets(*decomp, s_t, tool_info);

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  vtkSmartPointer<vtkPolyData> lines_for_polygon(const polygon_3& poly) {
    cout << "Getting lines" << endl;

    DBG_ASSERT(poly.vertices().size() > 2);

    // vtkSmartPointer<vtkPolyData> linesPolyData =
    //   vtkSmartPointer<vtkPolyData>::New();
 
    // // Create a vtkPoints container and store the points in it
    // vtkSmartPointer<vtkPoints> pts =
    //   vtkSmartPointer<vtkPoints>::New();
    // std::vector<point> verts = poly.vertices();

    // for (auto p : verts) {
    //   cout << "Point" << endl;
    //   pts->InsertNextPoint(p.x, p.y, p.z);
    // }
 
    // // Add the points to the polydata container
    // linesPolyData->SetPoints(pts);

    // cout << "Set points" << endl;
 
    // vtkSmartPointer<vtkCellArray> lines =
    //   vtkSmartPointer<vtkCellArray>::New();

    // for (vtkIdType i = 0; i < linesPolyData->GetNumberOfPoints(); i++) { 
    //   vtkSmartPointer<vtkLine> line0 =
    // 	vtkSmartPointer<vtkLine>::New();
    //   line0->GetPointIds()->SetId(0, i);
    //   line0->GetPointIds()->SetId(1, (i + 1) % linesPolyData->GetNumberOfPoints());

    //   lines->InsertNextCell(line0);
    // }
 
    // // Add the lines to the polydata container
    // linesPolyData->SetLines(lines);

    // Generate a 10 x 10 grid of points
    vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();
    for(unsigned int x = 0; x < 10; x++)
      {
	for(unsigned int y = 0; y < 10; y++)
	  {
	    points->InsertNextPoint(x + vtkMath::Random(-.25, .25),
				    y + vtkMath::Random(-.25,.25),
				    0);
	  }
      }
 
    vtkSmartPointer<vtkPolyData> aPolyData =
      vtkSmartPointer<vtkPolyData>::New();
    aPolyData->SetPoints(points);
 
    // Create a cell array to store the polygon in
    vtkSmartPointer<vtkCellArray> aCellArray =
      vtkSmartPointer<vtkCellArray>::New();
 
    // Define a polygonal hole with a clockwise polygon
    vtkSmartPointer<vtkPolygon> aPolygon =
      vtkSmartPointer<vtkPolygon>::New();
 
    aPolygon->GetPointIds()->InsertNextId(22);
    aPolygon->GetPointIds()->InsertNextId(23);
    aPolygon->GetPointIds()->InsertNextId(24);
    aPolygon->GetPointIds()->InsertNextId(25);
    aPolygon->GetPointIds()->InsertNextId(35);
    aPolygon->GetPointIds()->InsertNextId(45);
    aPolygon->GetPointIds()->InsertNextId(44);
    aPolygon->GetPointIds()->InsertNextId(43);
    aPolygon->GetPointIds()->InsertNextId(42);
    aPolygon->GetPointIds()->InsertNextId(32);
 
    aCellArray->InsertNextCell(aPolygon);
 
    // Create a polydata to store the boundary. The points must be the
    // same as the points we will triangulate.
    vtkSmartPointer<vtkPolyData> boundary =
      vtkSmartPointer<vtkPolyData>::New();
    boundary->SetPoints(aPolyData->GetPoints());
    boundary->SetPolys(aCellArray); 

    // Triangulate the grid points
    vtkSmartPointer<vtkDelaunay2D> delaunay =
      vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(aPolyData);
    delaunay->SetSourceData(boundary);
    delaunay->Update();

    auto act = polydata_actor(delaunay->GetOutput());

    visualize_actors({act});

    vtkSmartPointer<vtkPolyData> res = delaunay->GetOutput();
    return res;
  }

  // TODO: Dont just return bounding box
  triangular_mesh feature_mesh(const feature& f) {
    return extrude(f.base(), f.depth()*f.normal());
    // vector<point> pts = f.base().vertices();
    // concat(pts, f.top().vertices());

    // box b = bound_positions(pts);
    // auto tris = box_triangles(b);
    
    // return make_mesh(tris, 0.001);
    // cout << "Getting lines" << endl;

    // polygon_3 base = f.base();
    
    // //    auto line_polydata = lines_for_polygon(base);

    // vector<point> verts = base.vertices();
    // reverse(verts);
    // oriented_polygon p(base.normal(), verts);

    // auto pd = polydata_for_polygon(p);

    // vtkSmartPointer<vtkTriangleFilter> triangleFilter1 =
    //   vtkSmartPointer<vtkTriangleFilter>::New();
    // triangleFilter1->SetInputData(pd);
    // triangleFilter1->Update();
    
    // point n = f.normal();

    // cout << "Extruding" << endl;

    // vtkSmartPointer<vtkLinearExtrusionFilter> extrude = 
    //   vtkSmartPointer<vtkLinearExtrusionFilter>::New();
    // extrude->SetInputData(triangleFilter1->GetOutput());
    // extrude->SetExtrusionTypeToNormalExtrusion();
    // extrude->CappingOn();
    // extrude->SetVector(n.x, n.y, n.z);
    // extrude->SetScaleFactor(f.depth());
    // extrude->Update();

    // cout << "Done extruding" << endl;
 
    // vtkSmartPointer<vtkTriangleFilter> triangleFilter =
    //   vtkSmartPointer<vtkTriangleFilter>::New();
    // triangleFilter->SetInputConnection(extrude->GetOutputPort());
    // triangleFilter->Update();

    // auto res = trimesh_for_polydata(triangleFilter->GetOutput());

    // vtk_debug_mesh(res);

    // return res;
  }

  triangular_mesh subtract_features(const triangular_mesh& m,
				    feature_decomposition* features) {
    triangular_mesh subtracted = m;
    triangular_mesh* sub_ptr = &subtracted;
    auto subtract = [sub_ptr](feature* f) {
      if (f != nullptr) {
	triangular_mesh f_mesh = feature_mesh(*f);
	*sub_ptr = boolean_difference(*sub_ptr, f_mesh);
	vtk_debug_mesh(*sub_ptr);
      }
    };

    traverse_bf(features, subtract);

    vtk_debug_mesh(subtracted);

    return subtracted;
  }
  
  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
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

    for (auto& info : dir_info) {
      auto sfs = outer_surfaces(current_stock);
      auto orients = all_stable_orientations(sfs, v);
      point n = normal(info.decomp);
      clamp_orientation orient = find_orientation_by_normal(orients, n);
      fixture fix(orient, v);

      auto decomp = info.decomp;
      auto& acc_info = info.tool_info;

      delete_nodes(decomp, [acc_info](feature* f) {
	  return map_find(f, acc_info).size() == 0;
	});

      auto t = mating_transform(current_stock, orient, v);
      cut_setups.push_back(create_setup(t, current_stock, part, decomp, fix, info.tool_info));

      current_stock = subtract_features(current_stock, decomp);
      
      // plane clip_plane = face_plane(part, n);
      // current_stock = clip_mesh(current_stock, clip_plane);
    }

    return cut_setups;
  }
  
}

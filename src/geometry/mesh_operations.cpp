#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

#include "geometry/mesh_operations.h"

namespace gca {

  void debug_print_polydata(vtkPolyData* polydata) {
    cout << "# of points in polydata = " << polydata->GetNumberOfPoints() << endl;
    //    auto polys = polydata->GetPolys();
    cout << "# of polys in polydata = " << polydata->GetNumberOfPolys() << endl;
    for (vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++) {
      vtkCell* c = polydata->GetCell(i);
      cout << "Cell type = " << c->GetCellType() << endl;
      cout << "# of points = " << c->GetNumberOfPoints() << endl;
    }
  }

  triangular_mesh
  trimesh_from_polydata(vtkPolyData* in_polydata) {
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
      vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(in_polydata);
    triangleFilter->Update();

    vtkPolyData* polydata = triangleFilter->GetOutput();

    debug_print_polydata(polydata);
    
    assert(false);
  }

  vtkSmartPointer<vtkPolyData>
  polydata_for_trimesh(const triangular_mesh& mesh) {
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  for (auto i : mesh.vertex_indexes()) {
    point p = mesh.vertex(i);
    points->InsertNextPoint(p.x, p.y, p.z);
    i++;
  }

  vtkSmartPointer<vtkCellArray> triangles =
    vtkSmartPointer<vtkCellArray>::New();

  for (auto i : mesh.face_indexes()) {
    vtkSmartPointer<vtkTriangle> triangle =
      vtkSmartPointer<vtkTriangle>::New();

    auto t = mesh.triangle_vertices(i);
    triangle->GetPointIds()->SetId(0, t.v[0]);
    triangle->GetPointIds()->SetId(1, t.v[1]);
    triangle->GetPointIds()->SetId(2, t.v[2]);

    triangles->InsertNextCell(triangle);    
  }

  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  // Add the geometry and topology to the polydata
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  return polyData;
}
  
  triangular_mesh
  clip_mesh(const triangular_mesh& m,
	    const plane pl) {

    vtkSmartPointer<vtkPlane> clipPlane = 
      vtkSmartPointer<vtkPlane>::New();
    point n = pl.normal();
    point pt = pl.pt();
    clipPlane->SetNormal(n.x, n.y, n.z);
    clipPlane->SetOrigin(pt.x, pt.y, pt.z);

    vtkSmartPointer<vtkPolyData> input_mesh_data =
      polydata_for_trimesh(m);

    debug_print_polydata(input_mesh_data);

    // Clip the source with the plane
    vtkSmartPointer<vtkClipPolyData> clipper = 
      vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(input_mesh_data);
    clipper->SetClipFunction(clipPlane);
    clipper->Update();

    
    vtkPolyData* m_data = clipper->GetOutput();
					   
    return trimesh_from_polydata(m_data);
  }

}

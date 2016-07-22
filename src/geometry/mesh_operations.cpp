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

  // TODO: Actually compute normals
  triangle vtkCell_to_triangle(vtkCell* cell) {
    assert(cell->GetCellType() == 5);
    assert(cell->GetNumberOfPoints() == 3);
    auto pts = cell->GetPoints();
    double* v1 = pts->GetPoint(0);
    double* v2 = pts->GetPoint(0);
    double* v3 = pts->GetPoint(0);
    point normal(0, 0, 1);
    return triangle(normal,
		    point(v1[0], v1[1], v1[2]),
		    point(v2[0], v2[1], v2[2]),
		    point(v3[0], v3[1], v3[2]));
  }

  void debug_print_polydata(vtkPolyData* polydata) {
    cout << "# of points in polydata = " << polydata->GetNumberOfPoints() << endl;
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

    vector<triangle> tris;
    for (vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++) {
      vtkCell* c = polydata->GetCell(i);
      tris.push_back(vtkCell_to_triangle(c));
    }
    
    triangular_mesh m = make_mesh(tris, 0.001);
    assert(m.is_connected());
    return m;
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

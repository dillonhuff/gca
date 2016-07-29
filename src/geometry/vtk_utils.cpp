#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>

#include "geometry/vtk_utils.h"

namespace gca {

  triangle vtkCell_to_triangle(vtkCell* c) {
    assert(c->GetCellType() == VTK_TRIANGLE);
    assert(c->GetNumberOfPoints() == 3);

    vtkTriangle* tri = dynamic_cast<vtkTriangle*>(c);
    double p0[3];
    double p1[3];
    double p2[3];
    tri->GetPoints()->GetPoint(0, p0);
    tri->GetPoints()->GetPoint(1, p1);
    tri->GetPoints()->GetPoint(2, p2);

    point v1(p0[0], p0[1], p0[2]);
    point v2(p1[0], p1[1], p1[2]);
    point v3(p2[0], p2[1], p2[2]);

    point q1 = v1 - v3;
    point q2 = v2 - v3;
    point norm = cross(q2, q1).normalize();
    return triangle(norm, v1, v2, v3);
  }

  std::vector<triangle>
  polydata_to_triangle_list(vtkPolyData* in_polydata) {
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
      vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(in_polydata);
    triangleFilter->Update();

    vtkPolyData* polydata = triangleFilter->GetOutput();

    vector<triangle> tris;
    for (vtkIdType i = 0; i < polydata->GetNumberOfPolys(); i++) {
      vtkCell* c = polydata->GetCell(i);
      tris.push_back(vtkCell_to_triangle(c));
    }
    return tris;
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

}

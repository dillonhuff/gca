#include <vtkDecimatePro.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkStripper.h>
#include <vtkCutter.h>
#include <vtkFeatureEdges.h>
#include <vtkProperty.h>
#include <vtkFeatureEdges.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>
#include <vtkProperty.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>

#include <vtkAxesActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include "geometry/mesh_operations.h"
#include "geometry/surface.h"
#include "geometry/vtk_debug.h"

namespace gca {

  vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData) {
    vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
 
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
  }

  void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors) {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    // Create axes
    vtkSmartPointer<vtkAxesActor> axes =
      vtkSmartPointer<vtkAxesActor>::New();

    renderer->AddActor(axes);
    for (auto actor : actors) {
      renderer->AddActor(actor);
    }
    renderer->SetBackground(.1, .2, .3);

    vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindow->Render();
    renderWindowInteractor->Start();
  }

  // TODO: Actually compute normals
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

  triangular_mesh
  trimesh_from_polydata(vtkPolyData* in_polydata) {

    auto tris = polydata_to_triangle_list(in_polydata);
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

  // TOOD: Simplify this atrocity of a function
  triangular_mesh
  clip_mesh(const triangular_mesh& m,
	    const plane pl)
  {

    vtkSmartPointer<vtkPlane> clipPlane = 
      vtkSmartPointer<vtkPlane>::New();
    point n = pl.normal();
    point pt = pl.pt();
    clipPlane->SetNormal(n.x, n.y, n.z);
    clipPlane->SetOrigin(pt.x, pt.y, pt.z);

    vtkSmartPointer<vtkPolyData> input_mesh_data =
      polydata_for_trimesh(m);
    
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator->SetInputData(input_mesh_data);
    normalGenerator->SetSplitting(0);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    input_mesh_data = normalGenerator->GetOutput();
    
    // Clip the source with the plane
    vtkSmartPointer<vtkClipPolyData> clipper = 
      vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(input_mesh_data);
    clipper->SetClipFunction(clipPlane);
    clipper->Update();
    
    vtkPolyData* m_data = clipper->GetOutput();

    vector<triangle> main_tris = polydata_to_triangle_list(m_data);

    // Now extract clipped edges
    vtkSmartPointer<vtkFeatureEdges> boundaryEdges =
      vtkSmartPointer<vtkFeatureEdges>::New();
    boundaryEdges->SetInputData(m_data);
    boundaryEdges->BoundaryEdgesOn();
    boundaryEdges->FeatureEdgesOff();
    boundaryEdges->NonManifoldEdgesOff();
    boundaryEdges->ManifoldEdgesOff();
 
    vtkSmartPointer<vtkStripper> boundaryStrips =
      vtkSmartPointer<vtkStripper>::New();
    boundaryStrips->SetInputConnection(boundaryEdges->GetOutputPort());
    boundaryStrips->Update();
 
    // Change the polylines into polygons
    vtkSmartPointer<vtkPolyData> boundaryPoly =
      vtkSmartPointer<vtkPolyData>::New();
    boundaryPoly->SetPoints(boundaryStrips->GetOutput()->GetPoints());
    boundaryPoly->SetPolys(boundaryStrips->GetOutput()->GetLines());

    // Triangulate the polygons
    vtkSmartPointer<vtkTriangleFilter> triangleFilter =
      vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(boundaryPoly);
    triangleFilter->Update();

    vtkPolyData* patch_data = triangleFilter->GetOutput();

    vector<triangle> patch_tris = polydata_to_triangle_list(patch_data);
    concat(main_tris, patch_tris);

    triangular_mesh intermediate_mesh = make_mesh(main_tris, 0.01);

    auto pdata = polydata_for_trimesh(intermediate_mesh);

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator2 =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator2->SetInputData(pdata);
    normalGenerator2->SetSplitting(0);
    normalGenerator2->ComputePointNormalsOn();
    normalGenerator2->ComputeCellNormalsOn();
    normalGenerator2->Update();

    pdata = normalGenerator2->GetOutput();
    
    assert(is_closed(pdata));

    double target =
      1.0 - (static_cast<double>(pdata->GetNumberOfPolys()) - 12.0) / 12.0;
    vtkSmartPointer<vtkDecimatePro> decimate =
      vtkSmartPointer<vtkDecimatePro>::New();
    decimate->SetInputData(pdata);
    
    decimate->SetTargetReduction(target);
    decimate->Update();

    pdata = decimate->GetOutput();

    vtkSmartPointer<vtkPolyDataNormals> normalGenerator3 =
      vtkSmartPointer<vtkPolyDataNormals>::New();

    normalGenerator3->SetInputData(pdata);
    normalGenerator3->SetSplitting(0);
    normalGenerator3->ComputePointNormalsOn();
    normalGenerator3->ComputeCellNormalsOn();
    normalGenerator3->Update();

    pdata = normalGenerator3->GetOutput();
    
    assert(is_closed(pdata));
    assert(has_cell_normals(pdata));

    triangular_mesh final_mesh = trimesh_from_polydata(pdata);
    
    assert(final_mesh.is_connected());
    return final_mesh;
  }

}

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

#include "system/parse_stl.h"

void visualize_polydata(vtkSmartPointer<vtkPolyData> polyData) {
  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(polyData); //Connection(polyData->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  renderWindow->Render();
  renderWindowInteractor->Start();
}

vtkSmartPointer<vtkPolyData>
polydata_from_triangle_list(const std::vector<gca::triangle>& stl_triangles) {
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> triangles =
    vtkSmartPointer<vtkCellArray>::New();
  int i = 0;
  for (auto t : stl_triangles) {
    vtkSmartPointer<vtkTriangle> triangle =
      vtkSmartPointer<vtkTriangle>::New();
    
    points->InsertNextPoint (t.v1.x, t.v1.y, t.v1.z);
    triangle->GetPointIds()->SetId ( 0, i );
    i++;
    points->InsertNextPoint (t.v2.x, t.v2.y, t.v2.z);
    triangle->GetPointIds()->SetId ( 1, i );
    i++;
    points->InsertNextPoint (t.v3.x, t.v3.y, t.v3.z);
    triangle->GetPointIds()->SetId ( 2, i );
    i++;

    triangles->InsertNextCell(triangle);
  }
 
  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  // Add the geometry and topology to the polydata
  polyData->SetPoints ( points );
  polyData->SetPolys ( triangles );

  return polyData;
}

int main(int argc, char* argv[]) {
  assert(argc == 2);

  auto file = argv[1];
  auto stl_triangles = gca::parse_stl(file).triangles;
  auto polyData = polydata_from_triangle_list(stl_triangles);
  visualize_polydata(polyData);
 
  return EXIT_SUCCESS;
}

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>

#include "core/parser.h"
#include "simulators/sim_mill.h"
#include "system/bmp_output.h"

using namespace gca;
 
int main(int, char *[]) {
  context c;
  gprog* p = parse_gprog(c, "G1 X0 Y0 Z0 G91 G1 X3 Z4");
  region r(5, 5, 10, 0.05);
  r.set_height(0.1, 4.9, 0.1, 4.9, 9);
  r.set_machine_x_offset(1);
  r.set_machine_y_offset(3);
  double tool_diameter = 1.0;
  cylindrical_bit t(tool_diameter);
  simulate_mill(*p, r, t);

  int w = r.num_x_elems;
  int h = r.num_y_elems;

  // Create points on an XY grid with random Z coordinate
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  for(unsigned int x = 0; x < w; x++) {
    for(unsigned int y = 0; y < h; y++) {
      points->InsertNextPoint(x, y, r.column_height(x, y)); //vtkMath::Random(0.0, 3.0)); //
    }
  }

  vtkSmartPointer<vtkPolyData> polydata = 
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
  glyphFilter->SetInputConnection(polydata->GetProducerPort());
#else
  glyphFilter->SetInputData(polydata);
#endif
  glyphFilter->Update();
 
  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> pointsMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  pointsMapper->SetInputConnection(glyphFilter->GetOutputPort());
 
  vtkSmartPointer<vtkActor> pointsActor =
    vtkSmartPointer<vtkActor>::New();
  pointsActor->SetMapper(pointsMapper);
  pointsActor->GetProperty()->SetPointSize(3);
  pointsActor->GetProperty()->SetColor(1,0,0);
  // Triangulate the grid points
  vtkSmartPointer<vtkDelaunay2D> delaunay =
    vtkSmartPointer<vtkDelaunay2D>::New();
#if VTK_MAJOR_VERSION <= 5
  delaunay->SetInput(polydata);
#else
  delaunay->SetInputData(polydata);
#endif
  delaunay->Update();
 
  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  triangulatedMapper->SetInputConnection(delaunay->GetOutputPort());
 
  vtkSmartPointer<vtkActor> triangulatedActor =
    vtkSmartPointer<vtkActor>::New();
  triangulatedActor->SetMapper(triangulatedMapper);
 
  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  // Add the actor to the scene
  renderer->AddActor(pointsActor);
  renderer->AddActor(triangulatedActor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}

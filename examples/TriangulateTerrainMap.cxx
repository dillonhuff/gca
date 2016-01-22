// #include <vtkVersion.h>
// #include <vtkSmartPointer.h>
// #include <vtkProperty.h>
// #include <vtkPoints.h>
// #include <vtkPolyData.h>
// #include <vtkPointData.h>
// #include <vtkDelaunay2D.h>
// #include <vtkMath.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkActor.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderer.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkVertexGlyphFilter.h>

#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkDelaunay3D.h>
#include <vtkXMLPolyDataReader.h>

#include "core/parser.h"
#include "simulators/sim_mill.h"
#include "system/bmp_output.h"

using namespace gca;
 
int main(int, char *[]) {
  context c;
  gprog* p = parse_gprog(c, "G1 X0 Y0 Z0 G91 G1 X3 Z4 G1 X-3 Y-3 Z3 G0 Z-7 G0 X2 Y2 G1 Z-6");
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

  vtkSmartPointer<vtkDataSetMapper> originalMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  originalMapper->SetInputData(polydata);
 
  vtkSmartPointer<vtkActor> originalActor =
    vtkSmartPointer<vtkActor>::New();
  originalActor->SetMapper(originalMapper);
  originalActor->GetProperty()->SetColor(1,0,0);
 
  // Clean the polydata. This will remove duplicate points that may be
  // present in the input data.
  vtkSmartPointer<vtkCleanPolyData> cleaner =
    vtkSmartPointer<vtkCleanPolyData>::New();
  cleaner->SetInputData(polydata);
 
  // Generate a tetrahedral mesh from the input points. By
  // default, the generated volume is the convex hull of the points.
  vtkSmartPointer<vtkDelaunay3D> delaunay3D =
    vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay3D->SetInputConnection (cleaner->GetOutputPort());
 
  vtkSmartPointer<vtkDataSetMapper> delaunayMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  delaunayMapper->SetInputConnection(delaunay3D->GetOutputPort());
 
  vtkSmartPointer<vtkActor> delaunayActor =
    vtkSmartPointer<vtkActor>::New();
  delaunayActor->SetMapper(delaunayMapper);
  delaunayActor->GetProperty()->SetColor(1,0,0);
 
  // Generate a mesh from the input points. If Alpha is non-zero, then
  // tetrahedra, triangles, edges and vertices that lie within the
  // alpha radius are output.
  vtkSmartPointer<vtkDelaunay3D> delaunay3DAlpha =
    vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay3DAlpha->SetInputConnection (cleaner->GetOutputPort());
  delaunay3DAlpha->SetAlpha(0.1);
 
  vtkSmartPointer<vtkDataSetMapper> delaunayAlphaMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  delaunayAlphaMapper->SetInputConnection(delaunay3DAlpha->GetOutputPort());
 
  vtkSmartPointer<vtkActor> delaunayAlphaActor =
    vtkSmartPointer<vtkActor>::New();
  delaunayAlphaActor->SetMapper(delaunayAlphaMapper);
  delaunayAlphaActor->GetProperty()->SetColor(1,0,0);
 
  // Visualize
 
  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};
 
  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderer> delaunayRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderer> delaunayAlphaRenderer =
    vtkSmartPointer<vtkRenderer>::New();
 
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900,300);
 
  renderWindow->AddRenderer(originalRenderer);
  originalRenderer->SetViewport(leftViewport);
  renderWindow->AddRenderer(delaunayRenderer);
  delaunayRenderer->SetViewport(centerViewport);
  renderWindow->AddRenderer(delaunayAlphaRenderer);
  delaunayAlphaRenderer->SetViewport(rightViewport);
 
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  originalRenderer->AddActor(originalActor);
  delaunayRenderer->AddActor(delaunayActor);
  delaunayAlphaRenderer->AddActor(delaunayAlphaActor);
 
  originalRenderer->SetBackground(.3, .6, .3);
  delaunayRenderer->SetBackground(.4, .6, .3);
  delaunayAlphaRenderer->SetBackground(.5, .6, .3);
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  return EXIT_SUCCESS;
  
//   vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
//     vtkSmartPointer<vtkVertexGlyphFilter>::New();
// #if VTK_MAJOR_VERSION <= 5
//   glyphFilter->SetInputConnection(polydata->GetProducerPort());
// #else
//   glyphFilter->SetInputData(polydata);
// #endif
//   glyphFilter->Update();
 
//   // Create a mapper and actor
//   vtkSmartPointer<vtkPolyDataMapper> pointsMapper =
//     vtkSmartPointer<vtkPolyDataMapper>::New();
//   pointsMapper->SetInputConnection(glyphFilter->GetOutputPort());
 
//   vtkSmartPointer<vtkActor> pointsActor =
//     vtkSmartPointer<vtkActor>::New();
//   pointsActor->SetMapper(pointsMapper);
//   pointsActor->GetProperty()->SetPointSize(3);
//   pointsActor->GetProperty()->SetColor(1,0,0);
//   // Triangulate the grid points
//   vtkSmartPointer<vtkDelaunay2D> delaunay =
//     vtkSmartPointer<vtkDelaunay2D>::New();
// #if VTK_MAJOR_VERSION <= 5
//   delaunay->SetInput(polydata);
// #else
//   delaunay->SetInputData(polydata);
// #endif
//   delaunay->Update();
 
//   // Create a mapper and actor
//   vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper =
//     vtkSmartPointer<vtkPolyDataMapper>::New();
//   triangulatedMapper->SetInputConnection(delaunay->GetOutputPort());
 
//   vtkSmartPointer<vtkActor> triangulatedActor =
//     vtkSmartPointer<vtkActor>::New();
//   triangulatedActor->SetMapper(triangulatedMapper);
 
//   // Create a renderer, render window, and interactor
//   vtkSmartPointer<vtkRenderer> renderer =
//     vtkSmartPointer<vtkRenderer>::New();
//   vtkSmartPointer<vtkRenderWindow> renderWindow =
//     vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//     vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);
 
//   // Add the actor to the scene
//   renderer->AddActor(pointsActor);
//   renderer->AddActor(triangulatedActor);
//   renderer->SetBackground(.3, .6, .3); // Background color green
 
//   // Render and interact
//   renderWindow->Render();
//   renderWindowInteractor->Start();
 
//   return EXIT_SUCCESS;
}

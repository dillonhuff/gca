#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProgrammableFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
 
#include "gcode/parser.h"
#include "simulators/sim_mill.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[])
{
  // Create region and simulation
  arena_allocator a;
  set_system_allocator(&a);
  
  gprog* p = parse_gprog("G1 X0 Y0 Z0 G91 G1 X3 Z4");
  region r(5, 5, 7, 0.05);
  r.set_height(0.1, 4.9, 0.1, 4.9, 5);
  r.set_machine_x_offset(1);
  r.set_machine_y_offset(3);
  double tool_diameter = 1.0;
  cylindrical_bit t(tool_diameter);
  simulate_mill(*p, r, t);

  cout << "Done with simulation" << endl;

  int w = r.num_x_elems;
  int h = r.num_y_elems;

  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  for(unsigned int x = 0; x < w; x++) {
    for(unsigned int y = 0; y < h; y++) {
      double xd = static_cast<double>(x)*r.resolution;
      double yd = static_cast<double>(y)*r.resolution;
      points->InsertNextPoint(xd, yd, r.column_height(x, y));
    }
  }
 
  vtkSmartPointer<vtkPolyData> polyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  polyData->SetPoints(points);
 
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
  glyphFilter->SetInputConnection(polyData->GetProducerPort());
#else
  glyphFilter->SetInputData(polyData);
#endif
  glyphFilter->Update();
 
  // Visualize
 
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyphFilter->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
 
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}

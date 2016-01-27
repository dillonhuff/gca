#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
 
#include "core/parser.h"
#include "simulators/sim_mill.h"

using namespace gca;
using namespace std;

int main(int argc, char* argv[])
{
  // Create region and simulation
  arena_allocator a;
  set_system_allocator(&a);
  
  gprog* p = parse_gprog("G1 X0 Y0 Z0 G91 G1 X3 Z4");
  region r(5, 5, 10, 0.05);
  r.set_height(0.1, 4.9, 0.1, 4.9, 9);
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


// #include <vtkSmartPointer.h>
// #include <vtkObjectFactory.h>
// #include <vtkSphereSource.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkActor.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderer.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkProgrammableFilter.h>
// #include <vtkCommand.h>

// #include <vtkVersion.h>
// #include <vtkSmartPointer.h>
// #include <vtkPoints.h>
// #include <vtkVertexGlyphFilter.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkActor.h>
// #include <vtkRenderer.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>

// #include "core/parser.h"
// #include "simulators/sim_mill.h"

// using namespace gca;
// using namespace std;

// class CommandSubclass : public vtkCommand
// {
//   public:
//   vtkTypeMacro(CommandSubclass, vtkCommand);
//   static CommandSubclass *New()
//   {
//     return new CommandSubclass;
//   }
 
//   void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), 
//                        void *vtkNotUsed(callData))
//   {
//     std::cout << "timer callback" << std::endl;
 
//     vtkRenderWindowInteractor *iren = 
//       static_cast<vtkRenderWindowInteractor*>(caller);
 
//     this->ProgrammableFilter->Modified();
 
//     iren->Render();
 
//   }
 
//   vtkSmartPointer<vtkProgrammableFilter> ProgrammableFilter;
 
// };
 
// unsigned int counter; //global
// region* animate_region = NULL; // global

// void AdjustPoints(void* arguments) {
//   // std::cout << "AdjustPoints" << std::endl;
//   // vtkProgrammableFilter* programmableFilter = 
//   //     static_cast<vtkProgrammableFilter*>(arguments);
 
//   // vtkSmartPointer<vtkPoints> newPts =
//   //     vtkSmartPointer<vtkPoints>::New();

//   // vtkIdType nx = animate_region->num_x_elems;
//   // vtkIdType ny = animate_region->num_y_elems;
//   // double res = animate_region->resolution;
//   // vtkIdType numNewPts = nx*ny;
//   // newPts->SetNumberOfPoints(numNewPts);
 
//   // for(vtkIdType i = 0; i < nx; i++) {
//   //   for (vtkIdType j = 0; j < ny; j++) {
//   //     double p[3];
//   //     p[0] = static_cast<double>(i)*res;
//   //     p[1] = static_cast<double>(j)*res;
//   //     p[2] = animate_region->column_height(i, j);
//   //     newPts->SetPoint(i*nx + j, p);
//   //   }
//   // }
 
//   // programmableFilter->GetPolyDataOutput()->CopyStructure(programmableFilter->GetPolyDataInput());
//   // programmableFilter->GetPolyDataOutput()->SetPoints(newPts);

//   std::cout << "AdjustPoints" << std::endl;
//   vtkProgrammableFilter* programmableFilter = 
//     static_cast<vtkProgrammableFilter*>(arguments);
 
//   vtkPoints* inPts = programmableFilter->GetPolyDataInput()->GetPoints();
//   vtkIdType numPts = inPts->GetNumberOfPoints();
//   vtkSmartPointer<vtkPoints> newPts =
//     vtkSmartPointer<vtkPoints>::New();
//   newPts->SetNumberOfPoints(numPts);
 
//   for(vtkIdType i = 0; i < numPts; i++)
//     {
//       double p[3];
//       inPts->GetPoint(i, p);
//       newPts->SetPoint(i, p);
//     }
 
//   programmableFilter->GetPolyDataOutput()->CopyStructure(programmableFilter->GetPolyDataInput());
//   programmableFilter->GetPolyDataOutput()->SetPoints(newPts);  
//   counter++;
// }

// int main(int, char *[])
// {
//   // Create region and simulation
//   arena_allocator a;
//   set_system_allocator(&a);
  
//   gprog* p = parse_gprog("G1 X0 Y0 Z0 G91 G1 X3 Z4");
//   region r(5, 5, 10, 0.05);
//   animate_region = &r;
//   r.set_height(0.1, 4.9, 0.1, 4.9, 9);
//   r.set_machine_x_offset(1);
//   r.set_machine_y_offset(3);
//   double tool_diameter = 1.0;
//   cylindrical_bit t(tool_diameter);
//   simulate_mill(*p, r, t);

//   cout << "Done with simulation" << endl;

//   int w = r.num_x_elems;
//   int h = r.num_y_elems;

//   vtkSmartPointer<vtkPoints> points =
//     vtkSmartPointer<vtkPoints>::New();

//   for(unsigned int x = 0; x < w; x++) {
//     for(unsigned int y = 0; y < h; y++) {
//       points->InsertNextPoint(x, y, r.column_height(x, y)); //vtkMath::Random(0.0, 3.0)); //
//     }
//   }

//   vtkSmartPointer<vtkPolyData> polyData = 
//     vtkSmartPointer<vtkPolyData>::New();
//   polyData->SetPoints(points);

//   vtkSmartPointer<vtkProgrammableFilter> programmableFilter = 
//     vtkSmartPointer<vtkProgrammableFilter>::New();
//   programmableFilter->
//   programmableFilter->SetExecuteMethod(AdjustPoints, programmableFilter);
//   programmableFilter->Update();
 
//   // Visualize

//   vtkSmartPointer<vtkPolyDataMapper> mapper =
//     vtkSmartPointer<vtkPolyDataMapper>::New();
//   mapper->SetInputConnection(programmableFilter->GetOutputPort());

//   vtkSmartPointer<vtkActor> actor = 
//     vtkSmartPointer<vtkActor>::New();
//   actor->SetMapper(mapper);
 
//   // Create a renderer, render window, and interactor
//   vtkSmartPointer<vtkRenderer> renderer = 
//     vtkSmartPointer<vtkRenderer>::New();
//   vtkSmartPointer<vtkRenderWindow> renderWindow = 
//     vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
//     vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);
 
//   // Initialize must be called prior to creating timer events.
//   renderWindowInteractor->Initialize();
//   renderWindowInteractor->CreateRepeatingTimer(500);
 
//   vtkSmartPointer<CommandSubclass> timerCallback = 
//     vtkSmartPointer<CommandSubclass>::New();
//   timerCallback->ProgrammableFilter = programmableFilter;
 
//   renderWindowInteractor->AddObserver ( vtkCommand::TimerEvent, timerCallback );
 
//   // Add the actor to the scene
//   renderer->AddActor(actor);
//   renderer->SetBackground(1,1,1); // Background color white
 
//   // Render and interact
//   renderWindow->Render();
//   renderWindowInteractor->Start();
 
//   return EXIT_SUCCESS;
// }

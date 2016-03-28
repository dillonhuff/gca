#include <vtkSmartPointer.h>
#include <vtkCylinderSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include "analysis/gcode_to_cuts.h"
#include "core/lexer.h"
#include "geometry/point.h"

using namespace gca;

struct tool_simulator_data {
  unsigned i, j;
  vector<vector<cut*>>& cuts;
  point tool_pos;
  
  bool done() {
    if (!(i < cuts.size())) {
      return !(j < cuts[i].size());
    } else {
      return false;
    }
  }

  tool_simulator_data(vector<vector<cut*>>& cutsp) : i(0), j(0), cuts(cutsp) {}
  
  void update() {
    assert(!done());
    if (j < cuts[i].size()) {
      cout << "Updating tool_pos" << endl;
      auto t = cuts[i][j];
      tool_pos = t->get_end();
      j++;
    } else {
      i++;
      j = 0;
    }
  }

  point tool_position() const { return tool_pos; }
};

class vtkTimerCallback2 : public vtkCommand {
public:
  static vtkTimerCallback2 *New()
  {
    vtkTimerCallback2 *cb = new vtkTimerCallback2;
    cb->TimerCount = 0;
    return cb;
  }
 
  virtual void Execute(vtkObject *caller, unsigned long eventId,
		       void * vtkNotUsed(callData))
  {
    if (vtkCommand::TimerEvent == eventId)
      {
        ++this->TimerCount;
      }
    std::cout << this->TimerCount << std::endl;
    sim_data->update();
    point p = (0.1) * sim_data->tool_position();
    cout << "Tool position: " << p << endl;
    actor->SetPosition(p.x, p.y, p.z); //this->TimerCount, this->TimerCount, 0);
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();
  }
 
private:
  int TimerCount;
public:
  tool_simulator_data* sim_data;
  vtkActor* actor;
};

void run_simulator(tool_simulator_data& sim_data) {
  // Create a cylinder representing the tool
  vtkSmartPointer<vtkCylinderSource> toolSource = 
    vtkSmartPointer<vtkCylinderSource>::New();
  toolSource->SetCenter(0.0, 0.0, 0.0);
  toolSource->SetRadius(0.1);
  toolSource->SetHeight(2.0);
  toolSource->Update();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(toolSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
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
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
 
  // Render and interact
  renderWindow->Render();
 
  // Initialize must be called prior to creating timer events.
  renderWindowInteractor->Initialize();
 
  // Sign up to receive TimerEvent
  vtkSmartPointer<vtkTimerCallback2> cb = 
    vtkSmartPointer<vtkTimerCallback2>::New();
  cb->actor = actor;
  cb->sim_data = &sim_data;
  renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
 
  int timerId = renderWindowInteractor->CreateRepeatingTimer(1);
  std::cout << "timerId: " << timerId << std::endl;
 
  // Start the interaction and timer
  renderWindowInteractor->Start();
}
 
int main(int, char* []) {
  arena_allocator a;
  set_system_allocator(&a);
  
  string file_name = "/Users/dillon/Documents/PRL-Project-Folders/090318_1132/CAM/footboard bottom.NCF"; //"/Users/dillon/CppWorkspace/gca/CUT_INNER_RECT_3.ngc";
  std::ifstream t(file_name);
  std::string str((std::istreambuf_iterator<char>(t)),
		  std::istreambuf_iterator<char>());
  vector<block> p = lex_gprog(str);
  vector<vector<cut*>> cuts;
  auto r = gcode_to_cuts(p, cuts);
  if (r != GCODE_TO_CUTS_SUCCESS) {
    cout << "Error: " << r << endl;
    return 1;
  }
  tool_simulator_data sim_data(cuts);
  run_simulator(sim_data);
 
  return EXIT_SUCCESS;
}

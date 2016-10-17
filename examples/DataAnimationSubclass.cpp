#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkCellPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkObjectFactory.h>

#include "geometry/vtk_utils.h"
#include "synthesis/clamp_orientation.h"
#include "system/parse_stl.h"

using namespace gca;

void KeypressCallbackFunction(vtkObject* caller,
			      long unsigned int vtkNotUsed(eventId),
			      void* vtkNotUsed(clientData),
			      void* vtkNotUsed(callData) ) {
 
  vtkRenderWindowInteractor *iren = 
    static_cast<vtkRenderWindowInteractor*>(caller);
  // Close the window
  iren->GetRenderWindow()->Finalize();
 
  // Stop the interactor
  iren->TerminateApp();
  std::cout << "Closing window..." << std::endl;
}

// Catch mouse events
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle* New();

  int num_planes_selected;
  plane plane_list[3];

  vtkSmartPointer<vtkPolyData> Data;
  vtkSmartPointer<vtkDataSetMapper> selectedMapper;
  vtkSmartPointer<vtkActor> selectedActor;
  
  MouseInteractorStyle() {
    selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    selectedActor = vtkSmartPointer<vtkActor>::New();
  }

  void add_plane(vtkCellPicker& picker) {
    auto cell_id = picker.GetCellId();
    vtkCell* c = Data->GetCell(cell_id);
    triangle t = vtkCell_to_triangle(c);

    plane_list[num_planes_selected] = plane(normal(t), t.v1);
    
    num_planes_selected++;
  }
 
  virtual void OnLeftButtonDown() {
    // Get the location of the click (in window coordinates)
    int* pos = this->GetInteractor()->GetEventPosition();
 
    vtkSmartPointer<vtkCellPicker> picker =
      vtkSmartPointer<vtkCellPicker>::New();
    picker->SetTolerance(0.0005);
 
    // Pick from this location.
    picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());
 
    double* worldPosition = picker->GetPickPosition();
    std::cout << "Cell id is: " << picker->GetCellId() << std::endl;
 
    if(picker->GetCellId() != -1) {

      DBG_ASSERT(num_planes_selected < 3);

      add_plane(*picker);
 
      std::cout << "Pick position is: " << worldPosition[0] << " " << worldPosition[1]
		<< " " << worldPosition[2] << endl;
 
      vtkSmartPointer<vtkIdTypeArray> ids =
	vtkSmartPointer<vtkIdTypeArray>::New();
      ids->SetNumberOfComponents(1);
      ids->InsertNextValue(picker->GetCellId());
 
      vtkSmartPointer<vtkSelectionNode> selectionNode =
	vtkSmartPointer<vtkSelectionNode>::New();
      selectionNode->SetFieldType(vtkSelectionNode::CELL);
      selectionNode->SetContentType(vtkSelectionNode::INDICES);
      selectionNode->SetSelectionList(ids);
 
      vtkSmartPointer<vtkSelection> selection =
	vtkSmartPointer<vtkSelection>::New();
      selection->AddNode(selectionNode);
 
      vtkSmartPointer<vtkExtractSelection> extractSelection =
	vtkSmartPointer<vtkExtractSelection>::New();
      extractSelection->SetInputData(0, this->Data);
      extractSelection->SetInputData(1, selection);
      extractSelection->Update();
 
      // In selection
      vtkSmartPointer<vtkUnstructuredGrid> selected =
	vtkSmartPointer<vtkUnstructuredGrid>::New();
      selected->ShallowCopy(extractSelection->GetOutput());
 
      std::cout << "There are " << selected->GetNumberOfPoints()
		<< " points in the selection." << std::endl;
      std::cout << "There are " << selected->GetNumberOfCells()
		<< " cells in the selection." << std::endl;
 
      selectedMapper->SetInputData(selected);

      selectedActor->SetMapper(selectedMapper);
      selectedActor->GetProperty()->EdgeVisibilityOn();
      selectedActor->GetProperty()->SetEdgeColor(1,0,0);
      selectedActor->GetProperty()->SetLineWidth(3);
 
      this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(selectedActor);

    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }
 
};
 
vtkStandardNewMacro(MouseInteractorStyle);
 
int main(int, char *[]) {

  triangular_mesh m = parse_stl("./test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.0001);

  auto pd = polydata_for_trimesh(m);
 
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(pd);
 
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(0.5, 0.5, 0.5); //green
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkCallbackCommand> keypressCallback = 
    vtkSmartPointer<vtkCallbackCommand>::New();
  keypressCallback->SetCallback ( KeypressCallbackFunction );  

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);
  renderWindowInteractor->Initialize();
 
  // Set the custom stype to use for interaction.
  vtkSmartPointer<MouseInteractorStyle> style =
    vtkSmartPointer<MouseInteractorStyle>::New();
  style->SetDefaultRenderer(renderer);
  style->Data = pd;

  style->num_planes_selected = 0;
  
  renderWindowInteractor->SetInteractorStyle(style);
 
  renderer->AddActor(actor);
  renderer->ResetCamera();
 
  renderer->SetBackground(1, 1, 1); // Blue
 
  renderWindow->Render();
  renderWindowInteractor->Start();

  cout << "# of planes selected = " << style->num_planes_selected << endl;
  clamp_orientation orient(style->plane_list[0],
			   style->plane_list[1],
			   style->plane_list[2]);

  cout << "Part zero position = " << part_zero_position(orient) << endl;

  return EXIT_SUCCESS;
}

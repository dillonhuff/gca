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
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/visual_debug.h"
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

  vtkSmartPointer<vtkDataSetMapper> selectedMapper;
  vtkSmartPointer<vtkActor> selectedActor;
  
  MouseInteractorStyle() {
    selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    selectedActor = vtkSmartPointer<vtkActor>::New();
  }

  vtkPolyData* selected_polydata(vtkCellPicker& picker) {
    auto picked_actor = picker.GetActor();
    auto picked_mapper = picked_actor->GetMapper();
    vtkPolyData* picked_data =
      static_cast<vtkPolyData*>(picked_mapper->GetInput());

    return picked_data;
  }

  void add_plane(vtkCellPicker& picker) {
    auto cell_id = picker.GetCellId();
    auto picked_actor = picker.GetActor();
    auto picked_mapper = picked_actor->GetMapper();
    vtkPolyData* picked_data = selected_polydata(picker);

    vtkCell* c = picked_data->GetCell(cell_id);
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
      extractSelection->SetInputData(0, this->selected_polydata(*picker));
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

std::vector<vtkSmartPointer<vtkActor>>
fab_setup_actors(const fabrication_setup& setup) {
  auto vice_pd = polydata_for_vice(setup.v);
  auto vice_actor = polydata_actor(vice_pd);

  vector<vtkSmartPointer<vtkActor>> actors{vice_actor};
  auto a = setup.arrangement();
  for (auto n : a.mesh_names()) {
    if (a.metadata(n).display_during_debugging) {
      auto other_pd = polydata_for_trimesh(a.mesh(n));
      auto other_actor = polydata_actor(other_pd);

      if (n == "part") {
	other_actor->GetProperty()->SetOpacity(1.0);
      }
	
      actors.push_back(other_actor);
    }
  }

  cout << "# of actors = " << actors.size() << endl;

  return actors;
}

point gui_select_part_zero(const fabrication_setup& setup) {
  // auto pd = polydata_for_trimesh(arrangement.mesh("part"));

  // vtkSmartPointer<vtkPolyDataMapper> mapper =
  //   vtkSmartPointer<vtkPolyDataMapper>::New();
  // mapper->SetInputData(pd);

  // vtkSmartPointer<vtkActor> actor =
  //   vtkSmartPointer<vtkActor>::New();
  // actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
  // actor->SetMapper(mapper);

  auto actors = fab_setup_actors(setup);

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

  style->num_planes_selected = 0;
  
  renderWindowInteractor->SetInteractorStyle(style);
 
  //renderer->AddActor(actor);
  for (auto& actor : actors) {
    renderer->AddActor(actor);
  }
  renderer->ResetCamera();
 
  renderer->SetBackground(1, 1, 1); // Blue
 
  renderWindow->Render();
  renderWindowInteractor->Start();

  cout << "# of planes selected = " << style->num_planes_selected << endl;
  clamp_orientation orient(style->plane_list[0],
			   style->plane_list[1],
			   style->plane_list[2]);

  point part_zero = part_zero_position(orient);

  return part_zero;
}

int main(int, char *[]) {
  arena_allocator a;
  set_system_allocator(&a);

  vice test_v = custom_jaw_vice(6.0, 1.5, 10.0, point(0.0, 0.0, 0.0));
  vice test_vice = top_jaw_origin_vice(test_v);
    
  std::vector<plate_height> plates{0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(4.0, 4.0, 4.0, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.25);
  t1.set_cut_length(0.6);

  t1.set_shank_diameter(3.0 / 8.0);
  t1.set_shank_length(0.3);

  t1.set_holder_diameter(2.5);
  t1.set_holder_length(3.5);
    
  tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.5);
  t2.set_cut_length(0.3);

  t2.set_shank_diameter(0.5);
  t2.set_shank_length(0.5);

  t2.set_holder_diameter(2.5);
  t2.set_holder_length(3.5);

  tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
  t3.set_cut_diameter(0.12);
  t3.set_cut_length(1.2);

  t3.set_shank_diameter(0.5);
  t3.set_shank_length(0.05);

  t3.set_holder_diameter(2.5);
  t3.set_holder_length(3.5);

  tool t4{1.5, 3.94, 4, HSS, FLAT_NOSE};
  t4.set_cut_diameter(1.5);
  t4.set_cut_length(2.2);

  t4.set_shank_diameter(0.5);
  t4.set_shank_length(0.05);

  t4.set_holder_diameter(2.5);
  t4.set_holder_length(3.5);
    
  vector<tool> tools{t1, t2, t3, t4};
  
  triangular_mesh mesh =
    parse_stl("./test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.0001);

  fabrication_plan p =
    make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

  for (auto& step : p.steps()) {
    cout << "Part zero position = " << gui_select_part_zero(step) << endl;
  }

  return EXIT_SUCCESS;
}

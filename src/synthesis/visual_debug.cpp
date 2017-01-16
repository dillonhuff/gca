#include <vtkCellArray.h>
#include <vtkCellPicker.h>
#include <vtkExtractSelection.h>
#include <vtkLine.h>
#include <vtkProperty.h>
#include <vtkSelection.h>
#include <vtkUnstructuredGrid.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSelectionNode.h>

#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkIdTypeArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkObjectFactory.h>

#include "geometry/homogeneous_transformation.h"
#include "geometry/vtk_utils.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/visual_debug.h"

namespace gca {

  void append_polyline(unsigned npts,
		       vtkSmartPointer<vtkPolyData> linesPolyData,
		       vtkSmartPointer<vtkPoints> pts,
		       vtkSmartPointer<vtkCellArray> lines,
		       const polyline& pl) {
    for (auto p : pl) 
      {
	pts->InsertNextPoint(p.x, p.y, p.z);
      }

    if (pl.num_points() > 0) 
      {
	
    	for (vtkIdType i = 0; i < pl.num_points() - 1; i++) { 
    	  vtkSmartPointer<vtkLine> line0 =
    	    vtkSmartPointer<vtkLine>::New();
    	  line0->GetPointIds()->SetId(0, npts + i);
    	  line0->GetPointIds()->SetId(1, npts + i + 1);

    	  lines->InsertNextCell(line0);
    	}
      }
    
 
  }

  vtkSmartPointer<vtkActor>
  actor_for_toolpath(const toolpath& tp) {
    return polydata_actor(polydata_for_toolpath(tp));
  }
  
  vtkSmartPointer<vtkPolyData>
  polydata_for_toolpath(const toolpath& tp) {
    // Create the polydata where we will store all the geometric data
    vtkSmartPointer<vtkPolyData> linesPolyData =
      vtkSmartPointer<vtkPolyData>::New();
 
    // Create a vtkPoints container and store the points in it
    vtkSmartPointer<vtkPoints> pts =
      vtkSmartPointer<vtkPoints>::New();

    // Add the points to the polydata container
    linesPolyData->SetPoints(pts);
    vtkSmartPointer<vtkCellArray> lines =
      vtkSmartPointer<vtkCellArray>::New();

    // Add the lines to the polydata container
    linesPolyData->SetLines(lines);

    for (auto& pl : tp.lines()) {
      auto num_points_so_far = linesPolyData->GetNumberOfPoints();
      append_polyline(num_points_so_far, linesPolyData, pts, lines, pl);
    }

    cout << "# of lines = " << linesPolyData->GetNumberOfLines() << endl;

    return linesPolyData;
    
  }

  vtkSmartPointer<vtkPolyData>
  polydata_for_vice(const vice v) {
    box main = main_box(v);
    box upper_clamp = upper_clamp_box(v);
    box lower_clamp = lower_clamp_box(v);

    triangular_mesh main_mesh = make_mesh(box_triangles(main), 0.001);
    point top_normal(0, 0, 1);
    point p = max_point_in_dir(main_mesh, top_normal);

    plane top_plane_neg(-1*top_normal, p);

    
    triangular_mesh upper_mesh = make_mesh(box_triangles(upper_clamp), 0.001);
    point upper_normal(0, -1, 0);
    point upper_pt = max_point_in_dir(upper_mesh, upper_normal);

    plane upper_clamp_neg(-1*upper_normal, upper_pt);

    point right_normal(1, 0, 0);
    point right_pt = max_point_in_dir(upper_mesh, right_normal);

    plane right_plane_neg(right_normal, right_pt);
    
    triangular_mesh left_mesh = make_mesh(box_triangles(lower_clamp), 0.001);

    // auto p1_act = plane_actor(vtk_plane(top_plane_neg));
    // auto p2_act = plane_actor(vtk_plane(upper_clamp_neg));
    // auto p3_act = plane_actor(vtk_plane(right_plane_neg));

    // visualize_actors({p1_act, p2_act, p3_act});

    // auto bot = plane_actor(vtk_plane(v.base_plane()));
    // auto top = plane_actor(vtk_plane(v.top_jaw_plane()));
    // auto right = plane_actor(vtk_plane(v.right_bound_plane()));

    // visualize_actors({bot, top, right});
    
    // triangular_mesh left_mesh = make_mesh(box_triangles(lower_clamp), 0.001);
    // point left_normal(1, 0, 0);
    // point left_pt = max_point_in_dir(left_mesh, left_normal);

    // plane left_plane_neg(-1*left_normal, left_pt);

    // auto t = mate_planes(top_plane_neg.flip(), upper_clamp_neg.flip(), right_plane_neg.flip(),
    // 			 v.base_plane(), v.top_jaw_plane(), v.right_bound_plane());


    // NOTE: Finds transform, but it is reversed
    // auto t =
    //   mate_planes(v.base_plane(), v.top_jaw_plane(), v.right_bound_plane(),
    // 		  top_plane_neg.flip(), upper_clamp_neg.flip(), right_plane_neg.flip());

    // NOTE: Seems to be the same result as previous one
    auto t =
      mate_planes(top_plane_neg, upper_clamp_neg, right_plane_neg,
		  v.base_plane(), v.top_jaw_plane(), v.right_bound_plane());

    // NOTE: Segfaults
    // auto t =
    //   mate_planes(top_plane_neg.flip(), upper_clamp_neg.flip(), right_plane_neg.flip(),
    // 		  v.base_plane().flip(), v.top_jaw_plane().flip(), v.right_bound_plane().flip());
    
    
    DBG_ASSERT(t);

    vector<triangle> triangles;

    concat(triangles, apply(*t, main_mesh).triangle_list());
    concat(triangles, apply(*t, upper_mesh).triangle_list());
    concat(triangles, apply(*t, left_mesh).triangle_list());

    auto vice_pd = polydata_for_triangles(triangles);
    color_polydata(vice_pd, 0, 200, 0);

    return vice_pd;
  }

  void visual_debug(const fabrication_setup& setup) {
    auto vice_pd = polydata_for_vice(setup.v);
    auto vice_actor = polydata_actor(vice_pd);

    vector<vtkSmartPointer<vtkActor>> actors{vice_actor};
    auto a = setup.arrangement();
    for (auto n : a.mesh_names()) {
      if (a.metadata(n).display_during_debugging) {
	auto other_pd = polydata_for_trimesh(a.mesh(n));
	auto other_actor = polydata_actor(other_pd);

	if (n == "part") {
	  other_actor->GetProperty()->SetOpacity(0.5);
	}
	
	actors.push_back(other_actor);
      }
    }

    color white(255, 255, 255);

    for (auto& tp : setup.toolpaths()) {
      auto tp_polydata = polydata_for_toolpath(tp);
      color tp_color = random_color(white);
      color_polydata(tp_polydata, tp_color.red(), tp_color.green(), tp_color.blue());

      actors.push_back(polydata_actor(tp_polydata));
    }

    cout << "# of actors = " << actors.size() << endl;
    
    visualize_actors(actors);
  }

  void vtk_debug_toolpaths(const fabrication_setup& setup) {
    vector<vtkSmartPointer<vtkActor>> actors{};

    color white(255, 255, 255);

    for (auto& tp : setup.toolpaths()) {
      auto tp_polydata = polydata_for_toolpath(tp);
      color tp_color = random_color(white);
      color_polydata(tp_polydata, tp_color.red(), tp_color.green(), tp_color.blue());

      actors.push_back(polydata_actor(tp_polydata));
    }

    cout << "# of actors = " << actors.size() << endl;
    
    visualize_actors(actors);
  }
  
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
 
      if(picker->GetCellId() != -1) {

	DBG_ASSERT(num_planes_selected < 3);

	add_plane(*picker);
 
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

    return actors;
  }

  point gui_select_part_zero(const fabrication_setup& setup) {

    auto actors = fab_setup_actors(setup);

    vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkCallbackCommand> keypressCallback = 
      vtkSmartPointer<vtkCallbackCommand>::New();
    keypressCallback->SetCallback( KeypressCallbackFunction );

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
 
    for (auto& actor : actors) {
      renderer->AddActor(actor);
    }
    renderer->ResetCamera();
 
    renderer->SetBackground(1, 1, 1); // White
 
    renderWindow->Render();
    renderWindowInteractor->Start();

    cout << "# of planes selected = " << style->num_planes_selected << endl;
    clamp_orientation orient(style->plane_list[0],
			     style->plane_list[1],
			     style->plane_list[2]);

    point part_zero = part_zero_position(orient);

    return part_zero;
  }

}

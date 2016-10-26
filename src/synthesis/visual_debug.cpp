#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkProperty.h>

#include "geometry/homogeneous_transformation.h"
#include "geometry/vtk_utils.h"
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

  color random_color(const color mix) {
    unsigned red = rand() % 256;
    unsigned green = rand() % 256;
    unsigned blue = rand() % 256;

    // mix the color
    red = (red + mix.red()) / 2;
    green = (green + mix.green()) / 2;
    blue = (blue + mix.blue()) / 2;

    color color(red, green, blue);
    return color;    
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
  
}

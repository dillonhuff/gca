#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkProperty.h>

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

    for (auto& pl : tp.lines) {
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

    vector<triangle> triangles;
    concat(triangles, box_triangles(main));
    concat(triangles, box_triangles(upper_clamp));
    concat(triangles, box_triangles(lower_clamp));

    auto vice_pd = polydata_for_triangles(triangles);
    color_polydata(vice_pd, 0, 200, 0);

    return vice_pd;
  }

  class color {
  protected:
    unsigned r, g, b;

  public:
    color(unsigned p_r, unsigned p_g, unsigned p_b) :
      r(p_r), g(p_g), b(p_b) {}

    unsigned red() const { return r; }
    unsigned green() const { return r; }
    unsigned blue() const { return b; }
  };

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

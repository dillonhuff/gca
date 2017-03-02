#include "simulators/visual_debug.h"

#include <vtkCellArray.h>
#include <vtkVertex.h>

namespace gca {

  vtkSmartPointer<vtkPolyData>
  polydata_for_depth_field(const depth_field& df) {
    vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();
 
    vtkSmartPointer<vtkCellArray> vertices =
      vtkSmartPointer<vtkCellArray>::New();
    
    for (int i = 0; i < df.num_x_elems; i++) {
      for (int j = 0; j < df.num_y_elems; j++) {

	auto id = points->InsertNextPoint(df.x_center(i),
					  df.y_center(j),
					  df.column_height(i, j));
	vtkSmartPointer<vtkVertex> vertex = 
	  vtkSmartPointer<vtkVertex>::New();

	vertex->GetPointIds()->SetId(0, id);
	vertices->InsertNextCell(vertex);
      }
    }
 
    vtkSmartPointer<vtkPolyData> polydata =
      vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    polydata->SetVerts(vertices);

    return polydata;
  }

  void vtk_debug_depth_field(const depth_field& df) {
    auto pd_actor = polydata_actor(polydata_for_depth_field(df));

    visualize_actors({pd_actor});
  }

}

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

#include "geometry/triangular_mesh.h"
#include "system/parse_stl.h"

void visualize_polydata(vtkSmartPointer<vtkPolyData> polyData) {
  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(polyData); //Connection(polyData->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  renderer->AddActor(actor);
  renderer->SetBackground(.1, .2, .3); // Background color green
 
  renderWindow->Render();
  renderWindowInteractor->Start();
}

vtkSmartPointer<vtkPolyData>
polydata_from_triangle_list(const std::vector<gca::triangle>& stl_triangles) {
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> triangles =
    vtkSmartPointer<vtkCellArray>::New();
  int i = 0;
  for (auto t : stl_triangles) {
    vtkSmartPointer<vtkTriangle> triangle =
      vtkSmartPointer<vtkTriangle>::New();
    
    points->InsertNextPoint (t.v1.x, t.v1.y, t.v1.z);
    triangle->GetPointIds()->SetId ( 0, i );
    i++;
    points->InsertNextPoint (t.v2.x, t.v2.y, t.v2.z);
    triangle->GetPointIds()->SetId ( 1, i );
    i++;
    points->InsertNextPoint (t.v3.x, t.v3.y, t.v3.z);
    triangle->GetPointIds()->SetId ( 2, i );
    i++;

    triangles->InsertNextCell(triangle);
  }
 
  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  // Add the geometry and topology to the polydata
  polyData->SetPoints ( points );
  polyData->SetPolys ( triangles );

  return polyData;
}

void color_polydata(vtkSmartPointer<vtkPolyData> polyData,
		    const gca::triangular_mesh& mesh) {
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
 
  std::cout << "There are " << polyData->GetNumberOfPoints()
            << " points." << std::endl;
 
  for(int i = 0; i < polyData->GetNumberOfPoints(); i++) {
    double p[3];
    polyData->GetPoint(i, p);
    gca::point pt(p[0], p[1], p[2]);
    unsigned char color[3];
    if (mesh.is_constant_orientation_vertex(pt, 180.0)) {
      color[0] = 200;
    } else {
      color[0] = 0;
    }
    color[1] = 0;
    color[2] = 0;

    // for(unsigned int j = 0; j < 3; j++) {
    //   if (j == 0) {
    // 	color[j] = p[2] > 0.5 ? 200 : 0; //static_cast<unsigned char>(255.0 * dcolor[j]);}
    //   } else {
    // 	color[j] = 0;
    //   }
    // }
    std::cout << "color: "
	      << (int)color[0] << " "
	      << (int)color[1] << " "
	      << (int)color[2] << std::endl;
 
#if VTK_MAJOR_VERSION < 7
    colors->InsertNextTupleValue(color);
#else
    colors->InsertNextTypedTuple(color);
#endif
  }
 
  polyData->GetPointData()->SetScalars(colors);  
}

int main(int argc, char* argv[]) {
  assert(argc == 2);

  auto file = argv[1];
  auto stl_triangles = gca::parse_stl(file).triangles;
  auto mesh = make_mesh(stl_triangles, 0.0001);
  assert(mesh.is_connected());
  auto tl_list = mesh.triangle_list();
  auto polyData = polydata_from_triangle_list(tl_list);
  color_polydata(polyData, mesh);
  visualize_polydata(polyData);
 
  return EXIT_SUCCESS;
}

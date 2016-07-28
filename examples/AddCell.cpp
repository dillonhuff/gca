#include <vtkAxesActor.h>
#include <vtkCellData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCylinderSource.h>
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
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/millability.h"
#include "synthesis/vice.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;

vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData) {
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(polyData);
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  return actor;
}

void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors) {
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    // Create axes
  vtkSmartPointer<vtkAxesActor> axes =
    vtkSmartPointer<vtkAxesActor>::New();

  renderer->AddActor(axes);
  for (auto actor : actors) {
    renderer->AddActor(actor);
  }
  renderer->SetBackground(.1, .2, .3);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();
  renderWindowInteractor->Start();
}

void visualize_polydata(vtkSmartPointer<vtkPolyData> polyData) {
  auto actor = polydata_actor(polyData);
  vector<vtkSmartPointer<vtkActor> > actors{actor};
  visualize_actors(actors);
}

vtkSmartPointer<vtkPolyData>
polydata_for_trimesh(const triangular_mesh& mesh) {
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();

  for (auto i : mesh.vertex_indexes()) {
    point p = mesh.vertex(i);
    points->InsertNextPoint(p.x, p.y, p.z);
    i++;
  }

  vtkSmartPointer<vtkCellArray> triangles =
    vtkSmartPointer<vtkCellArray>::New();

  for (auto i : mesh.face_indexes()) {
    vtkSmartPointer<vtkTriangle> triangle =
      vtkSmartPointer<vtkTriangle>::New();

    auto t = mesh.triangle_vertices(i);
    triangle->GetPointIds()->SetId(0, t.v[0]);
    triangle->GetPointIds()->SetId(1, t.v[1]);
    triangle->GetPointIds()->SetId(2, t.v[2]);

    triangles->InsertNextCell(triangle);    
  }

  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  // Add the geometry and topology to the polydata
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  return polyData;
}

void color_polydata_by_millability(vtkSmartPointer<vtkPolyData> polyData,
				   const gca::triangular_mesh& mesh) {
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
 
  std::cout << "There are " << polyData->GetNumberOfPoints()
            << " points." << std::endl;

  vector<index_t> millable = millable_faces(point(0, -1, 0), mesh);
  for(index_t i = 0; i < polyData->GetNumberOfCells(); i++) {
    unsigned char color[3];
    if (elem(i, millable)) {
      color[0] = 200;
    } else {
      color[0] = 0;
    }
    color[1] = 0;
    color[2] = 0;

    colors->InsertNextTupleValue(color);
  }
 
  polyData->GetCellData()->SetScalars(colors);
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
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  return polyData;
}

vtkSmartPointer<vtkPolyData>
polydata_from_vice(const vice v) {
  box main = main_box(v);
  box upper_clamp = upper_clamp_box(v);
  box lower_clamp = lower_clamp_box(v);

  vector<triangle> triangles;
  concat(triangles, box_triangles(main));
  concat(triangles, box_triangles(upper_clamp));
  concat(triangles, box_triangles(lower_clamp));
  return polydata_from_triangle_list(triangles);
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
    color[0] = 200;
    color[1] = 0;
    color[2] = 0;

    colors->InsertNextTupleValue(color);
  }
 
  polyData->GetPointData()->SetScalars(colors);  
}

int main(int argc, char* argv[]) {
  assert(argc == 2);

  auto file = argv[1];
  auto stl_triangles = gca::parse_stl(file).triangles;
  auto mesh = make_mesh(stl_triangles, 0.0001);
  assert(mesh.is_connected());

  auto to_render = mesh;

  auto poly_data = polydata_for_trimesh(to_render);
  color_polydata_by_millability(poly_data, to_render);
  auto poly_actor = polydata_actor(poly_data);

  vector<vtkSmartPointer<vtkActor>> actors{poly_actor};
  visualize_actors(actors);
 
  return EXIT_SUCCESS;
}

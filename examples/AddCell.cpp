#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/millability.h"
#include "synthesis/vice.h"
#include "utils/algorithm.h"
#include "system/parse_stl.h"

using namespace gca;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  auto file = argv[1];
  auto stl_triangles = gca::parse_stl(file).triangles;
  auto mesh = make_mesh(stl_triangles, 0.0001);

  DBG_ASSERT(mesh.is_connected());
  
  box bounding = mesh.bounding_box();

  cout << "Bounding box = " << endl;
  cout << bounding << endl;
  cout << "X len = " << bounding.x_len() << endl;
  cout << "Y len = " << bounding.y_len() << endl;
  cout << "Z len = " << bounding.z_len() << endl;

  auto to_render = mesh;

  auto poly_data = polydata_for_trimesh(to_render);
  auto poly_actor = polydata_actor(poly_data);

  vector<vtkSmartPointer<vtkActor>> actors{poly_actor};
  visualize_actors(actors);
 
  return EXIT_SUCCESS;
}

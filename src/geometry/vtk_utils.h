#ifndef GCA_VTK_UTILS_H
#define GCA_VTK_UTILS_H

#include <vtkPlane.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<triangle>
  polydata_to_triangle_list(vtkPolyData* in_polydata);
  
  vtkSmartPointer<vtkPolyData>
  polydata_for_trimesh(const triangular_mesh& mesh);

  triangular_mesh
  trimesh_for_polydata(vtkPolyData* in_polydata);


  std::vector<triangle>
  vtk_triangulate_poly(const oriented_polygon& p);

  vtkSmartPointer<vtkPolyData>
  polydata_for_triangles(const std::vector<triangle>& tris);

  vtkSmartPointer<vtkPolyData>
  polydata_for_polygon(const oriented_polygon& p);

  vtkSmartPointer<vtkPlane> vtk_plane(const plane p);  

  line vtkCell_to_line(vtkCell* c);
}

#endif

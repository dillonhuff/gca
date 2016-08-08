#ifndef GCA_VTK_DEBUG_H
#define GCA_VTK_DEBUG_H

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "geometry/rigid_arrangement.h"
#include "geometry/surface.h"

namespace gca {

  bool has_cell_normals(vtkPolyData* polydata);
  bool is_closed(vtkPolyData* polydata);
  
  void debug_print_summary(vtkPolyData* polydata);
  void debug_print_edge_summary(vtkPolyData* pdata);
  void debug_print_is_closed(vtkPolyData* polydata);
  void debug_print_polydata(vtkPolyData* polydata);

  vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData);
  void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors);

  void vtk_debug_triangles(const std::vector<triangle>& mesh);
  void vtk_debug_highlight_inds(const std::vector<index_t>& inds,
				const triangular_mesh& mesh);
  void vtk_debug_mesh(const triangular_mesh& mesh);
  void vtk_debug_meshes(const std::vector<const triangular_mesh*>& mesh);
  void vtk_debug_highlight_inds(const surface& surf);
  void vtk_debug_highlight_inds(const std::vector<surface>& surfs);

  void vtk_debug_polygon(const oriented_polygon& p);
  void vtk_debug_polygons(const std::vector<oriented_polygon>& polys);

  void vtk_debug_mesh_boundary_edges(const triangular_mesh& m);
  void debug_arrangement(const rigid_arrangement& a);
}

#endif

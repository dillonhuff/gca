#ifndef GCA_VTK_DEBUG_H
#define GCA_VTK_DEBUG_H

#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "geometry/triangular_mesh.h"

namespace gca {

  bool has_cell_normals(vtkPolyData* polydata);
  bool is_closed(vtkPolyData* polydata);
  
  void debug_print_summary(vtkPolyData* polydata);
  void debug_print_edge_summary(vtkPolyData* pdata);
  void debug_print_is_closed(vtkPolyData* polydata);
  void debug_print_polydata(vtkPolyData* polydata);

  vtkSmartPointer<vtkActor> polydata_actor(vtkSmartPointer<vtkPolyData> polyData);
  void visualize_actors(const std::vector<vtkSmartPointer<vtkActor> >& actors);
  
  void vtk_debug_highlight_inds(const std::vector<index_t>& inds,
				const triangular_mesh& mesh);
}

#endif

#ifndef GCA_VTK_DEBUG_H
#define GCA_VTK_DEBUG_H

#include <vtkPolyData.h>

namespace gca {

  bool has_cell_normals(vtkPolyData* polydata);
  bool is_closed(vtkPolyData* polydata);
  
  void debug_print_summary(vtkPolyData* polydata);
  void debug_print_edge_summary(vtkPolyData* pdata);
  void debug_print_is_closed(vtkPolyData* polydata);
  void debug_print_polydata(vtkPolyData* polydata);

}

#endif

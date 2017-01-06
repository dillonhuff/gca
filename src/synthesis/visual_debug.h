#ifndef GCA_VISUAL_DEBUG_H
#define GCA_VISUAL_DEBUG_H

#include "geometry/vtk_debug.h"
#include "synthesis/fabrication_plan.h"

namespace gca {

  void visual_debug(const fabrication_setup& setup);

  vtkSmartPointer<vtkPolyData>
  polydata_for_vice(const vice v);

  vtkSmartPointer<vtkPolyData>
  polydata_for_toolpath(const toolpath& tp);

  vtkSmartPointer<vtkActor>
  actor_for_toolpath(const toolpath& tp);
  
  point gui_select_part_zero(const fabrication_setup& setup);

}

#endif

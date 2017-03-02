#pragma once

#include "geometry/vtk_debug.h"
#include "simulators/region.h"

namespace gca {

  vtkSmartPointer<vtkPolyData>
  polydata_for_depth_field(const depth_field& df);

  void vtk_debug_depth_field(const depth_field& df);

}

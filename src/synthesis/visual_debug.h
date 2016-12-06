#ifndef GCA_VISUAL_DEBUG_H
#define GCA_VISUAL_DEBUG_H

#include "geometry/vtk_debug.h"
#include "synthesis/fabrication_plan.h"

namespace gca {

  class color {
  protected:
    unsigned r, g, b;

  public:
    color(unsigned p_r, unsigned p_g, unsigned p_b) :
      r(p_r), g(p_g), b(p_b) {}

    unsigned red() const { return r; }
    unsigned green() const { return r; }
    unsigned blue() const { return b; }
  };

  void visual_debug(const fabrication_setup& setup);

  vtkSmartPointer<vtkPolyData>
  polydata_for_vice(const vice v);

  color random_color(const color mix);

  vtkSmartPointer<vtkPolyData>
  polydata_for_toolpath(const toolpath& tp);

  vtkSmartPointer<vtkActor>
  actor_for_toolpath(const toolpath& tp);
  
  point gui_select_part_zero(const fabrication_setup& setup);

}

#endif

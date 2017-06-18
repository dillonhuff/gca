#include "catch.hpp"

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkIntArray.h>
#include <vtkDataArray.h>
#include <vtkOBBTree.h>

#include "geometry/voxel_volume.h"
#include "geometry/voxel_volume_debug.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "system/parse_stl.h"

namespace gca {

  point min_point(const box b) {
    return point(b.x_min, b.y_min, b.z_min);
  }

  bool contains_point_obb(const point pt,
			  vtkSmartPointer<vtkOBBTree>& tree) {
 
    // Intersect the locator with the line
    double lineP0[3] = {pt.x, pt.y, pt.z};
    double lineP1[3] = {-10000.0, -1000.0, -1000.0};
    double lineP2[3] = {100.0, 100.0, 100.0};

    vtkSmartPointer<vtkPoints> intersectPoints = 
      vtkSmartPointer<vtkPoints>::New();
 
    int res = tree->IntersectWithLine(lineP0, lineP1, intersectPoints, NULL);
    if (res != -1) {
      return false;
    }

    int res2 = tree->IntersectWithLine(lineP0, lineP2, intersectPoints, NULL);

    if (res == -1 && res2 == -1) {
      return true;
    } else {
      //cout << "res = " << res << endl;
      //assert(res == 1);
      return false;
    }

 
    // std::cout << "NumPoints: " << intersectPoints->GetNumberOfPoints()
    // 	      << std::endl;
 
    // // Display list of intersections
    // double intersection[3];
    // for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
    //   {
    // 	intersectPoints->GetPoint(i, intersection);
    // 	std::cout << "Intersection " << i << ": "
    // 		  << intersection[0] << ", "
    // 		  << intersection[1] << ", "
    // 		  << intersection[2] << std::endl;
    //   }
     
  }

  bool contains_point(const point p,
		      vtkSmartPointer<vtkPolyData>& pd) {
    double testInside[3] = {p.x, p.y, p.z};

    vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(testInside);
 
    vtkSmartPointer<vtkPolyData> pointsPolydata = 
      vtkSmartPointer<vtkPolyData>::New();
    pointsPolydata->SetPoints(points);
 
    //Points inside test
    vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = 
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    selectEnclosedPoints->SetInputData(pointsPolydata);
    selectEnclosedPoints->SetSurfaceData(pd);
    selectEnclosedPoints->Update();
 
    return selectEnclosedPoints->IsInside(0);
  }

  voxel_volume build_from_mesh(const triangular_mesh& m) {
    box bb = m.bounding_box();
    double resolution = bb.x_len() / 80.0;

    auto pd = polydata_for_trimesh(m);
    
    DBG_ASSERT(is_closed(pd));

    vtkSmartPointer<vtkOBBTree> tree = 
      vtkSmartPointer<vtkOBBTree>::New();
    tree->SetDataSet(pd);
    tree->BuildLocator();
    
    voxel_volume vv(min_point(bb), bb.x_len(), bb.y_len(), bb.z_len(), resolution);

    for (int i = 0; i < vv.num_x_elems(); i++) {
      for (int j = 0; j < vv.num_y_elems(); j++) {
	for (int k = 0; k < vv.num_z_elems(); k++) {

	  point pt(vv.x_center(i), vv.y_center(j), vv.z_center(k));
	  if (contains_point_obb(pt, tree)) {
	    vv.set_occupied(i, j, k);
	  }

	}
      }
    }
    
    return vv;
  }

  TEST_CASE("Initial voxel volume is empty") {
    voxel_volume vol(point(0, 0, 0), 1.0, 1.0, 1.0, 0.1);

    REQUIRE(vol.is_empty(0, 0, 0));
  }


  TEST_CASE("After setting the voxel it is occupied") {
    voxel_volume vol(point(0, 0, 0), 1.0, 1.0, 1.0, 0.1);

    vol.set_occupied(0, 0, 0);

    REQUIRE(vol.is_occupied(0, 0, 0));

  }

  TEST_CASE("Loading a model from an stl file") {
    triangular_mesh m =
      //parse_stl("test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);
      parse_stl("test/stl-files/onshape_parts/Magnetic Latch Top - Part 1.stl", 0.0001);
    voxel_volume vv = build_from_mesh(m);

    vtk_debug_voxel_volume(vv);
  }
  
}

#include "catch.hpp"

#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/mandatory_volumes.h"
#include "synthesis/visual_debug.h"
#include "system/file.h"
#include "system/parse_stl.h"

namespace gca {

    vector<string> test_parts{
      "test/stl-files/onshape_parts/Part Studio 1 - Part 1(3).stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1(20).stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", 
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 
	"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 
	"test/stl-files/OctagonWithHolesShort.stl",
	"test/stl-files/CircleWithFilletAndSide.stl",
	"test/stl-files/onshape_parts/100-013 - Part 1.stl",
	"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl",
	"test/stl-files/onshape_parts/Japanese_Two_Contours_Part.stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl",
	"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl"};

  Nef_polyhedron compute_negative_space(const triangular_mesh& mesh) {
    box bb = mesh.bounding_box();
    double eps = 0.000001;
    bb.x_min += eps;
    bb.y_min += eps;
    bb.z_min += eps;

    bb.x_max -= eps;
    bb.y_max -= eps;
    bb.z_max -= eps;

    triangular_mesh stock = make_mesh(box_triangles(bb), 0.001);

    //vtk_debug_meshes({mesh, stock});

    Nef_polyhedron mesh_nef = trimesh_to_nef_polyhedron(mesh);
    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    auto negative_space = stock_nef - mesh_nef;

    return negative_space;
  }

  void compute_negative_space_file(const std::string& n) {
    if (!ends_with(n, "stl")) {
      return;
    }

    auto mesh = parse_stl(n, 0.0001);

    auto negative_space = compute_negative_space(mesh);

    cout << "about to convert to trimeshes" << endl;
    auto neg_meshes = nef_polyhedron_to_trimeshes(negative_space);

    vtk_debug_meshes(neg_meshes);
    
  }

  TEST_CASE("Negative space for all onshape parts") {
    read_dir("./test/stl-files/onshape_parts/", compute_negative_space_file, ".stl");
  }
  
  TEST_CASE("Subtracting from ") {

    auto mesh = parse_stl("./test/stl-files/onshape_parts/Part Studio 1 - Part 1(10).stl", 0.0001);

    auto mvs = mandatory_volumes(mesh);

    vector<triangular_mesh> mvs_meshes;
    for (auto& mv : mvs) {
      mvs_meshes.push_back(mv.front().volume);
    }

    vtk_debug_meshes(mvs_meshes);

    auto negative_space = compute_negative_space(mesh);

    cout << "about to convert to trimeshes" << endl;
    auto neg_meshes = nef_polyhedron_to_trimeshes(negative_space);
  }

}

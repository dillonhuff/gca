#include "catch.hpp"
#include "geometry/extrusion.h"
#include "geometry/vtk_debug.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Layered extrusion") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Extrude single layer box") {
      point extrude_dir(1, 0, 0);
      std::vector<point> pts{point(0, 0, 0), point(0, 1, 0), point(0, 1, 1), point(0, 0, 1)};
      std::vector<index_poly> polys{{0, 1, 2, 3}};
      std::vector<double> depths{3.4};
      triangular_mesh m = extrude_layers(pts, polys, depths, extrude_dir);
      auto pd = polydata_for_trimesh(m);
      debug_print_summary(pd);
      debug_print_is_closed(pd);
      debug_print_edge_summary(pd);

      vector<gca::edge> nm_edges = non_manifold_edges(m);
      cout << "# of vertices = " << m.vertex_indexes().size() << endl;
      cout << "# of faces = " << m.face_indexes().size() << endl;
      cout << "# of edges = " << m.edges().size() << endl;
      cout << "Non manifold edges = " << endl;
      for (auto e : nm_edges) {
	cout << " of adjacent triangles: " << m.edge_face_neighbors(e).size() << endl;
      }

      REQUIRE(nm_edges.size() == 0);

      REQUIRE(m.winding_order_is_consistent());

      //vtk_debug_mesh(m);

      REQUIRE(m.face_indexes().size() == 12);
      REQUIRE(m.is_connected());
    }

    SECTION("Extrude 2 layer jaw") {
      point extrude_dir(0, 1, 0);

      std::vector<point> pts{
	point(0, 0, 0),
	  point(0, 0, 10),
	  point(10, 0, 10),
	  point(10, 0, 9),
	  point(7, 0, 7),
	  point(10, 0, 5),
	  point(10, 0, 0)};

      std::vector<index_poly> polys{{0, 1, 2, 3, 4, 5, 6}, {0, 1, 2, 3, 5, 6}};
      std::vector<double> depths{2.3, 3.6};
      triangular_mesh m = extrude_layers(pts, polys, depths, extrude_dir);
      auto pd = polydata_for_trimesh(m);
      debug_print_summary(pd);
      debug_print_is_closed(pd);
      debug_print_edge_summary(pd);

      vector<gca::edge> nm_edges = non_manifold_edges(m);
      cout << "# of vertices = " << m.vertex_indexes().size() << endl;
      cout << "# of faces = " << m.face_indexes().size() << endl;
      cout << "# of edges = " << m.edges().size() << endl;
      cout << "Non manifold edges = " << endl;
      for (auto e : nm_edges) {
	cout << " of adjacent triangles: " << m.edge_face_neighbors(e).size() << endl;
      }

      REQUIRE(nm_edges.size() == 0);

      REQUIRE(m.winding_order_is_consistent());

      //      vtk_debug_mesh(m);

      REQUIRE(m.is_connected());
    }

  }
}

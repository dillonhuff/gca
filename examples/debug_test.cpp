#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "geometry/extrusion.h"
#include "geometry/vtk_debug.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Unlayered extrusion with no holes") {
    arena_allocator a;
    set_system_allocator(&a);

    point extrude_vector(1, 0, 0);
    std::vector<point> pts{point(0, 0, 0), point(0, 1, 0), point(0, 1, 1), point(0, 0, 1)};
    polygon_3 poly(pts);

    triangular_mesh m = extrude(poly, extrude_vector);

    vector<gca::edge> nm_edges = non_manifold_edges(m);
    cout << "# of vertices = " << m.vertex_indexes().size() << endl;
    cout << "# of faces = " << m.face_indexes().size() << endl;
    cout << "# of edges = " << m.edges().size() << endl;
    cout << "Non manifold edges = " << endl;
    for (auto e : nm_edges) {
      cout << e << " of adjacent triangles: " << m.edge_face_neighbors(e).size() << endl;
    }
    
    REQUIRE(nm_edges.size() == 0);

    REQUIRE(m.winding_order_is_consistent());

    //vtk_debug_mesh(m);

    REQUIRE(m.face_indexes().size() == 12);
    REQUIRE(m.is_connected());

    auto outer_surfs = outer_surfaces(m);

    REQUIRE(outer_surfs.size() == 6);

  }

}

#include "catch.hpp"
#include "geometry/extrusion.h"
#include "geometry/vtk_debug.h"
#include "system/parse_stl.h"
#include "utils/arena_allocator.h"

namespace gca {

  TEST_CASE("Extrude single layer form") {
    arena_allocator a;
    set_system_allocator(&a);

    point extrude_dir(1, 0, 0);
    std::vector<point> pts{point(0, 0, 0), point(0, 1, 0), point(0, 1, 1), point(0, 0, 1)};
    std::vector<index_poly> polys{{0, 1, 2, 3}};
    std::vector<double> depths{3.4};
    triangular_mesh m = extrude_layers(pts, polys, depths, extrude_dir);
    auto pd = polydata_for_trimesh(m);
    debug_print_summary(pd);
    debug_print_is_closed(pd);
    debug_print_edge_summary(pd);

    REQUIRE(non_manifold_edges(m).size() == 0);
    REQUIRE(m.winding_order_is_consistent());

    vtk_debug_mesh(m);

    REQUIRE(m.face_indexes().size() == 12);
    REQUIRE(m.is_connected());

  }
}

#include "geometry/octree.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

using namespace gca;

struct list_trimesh {
  std::vector<triangle_t> tris;
  std::vector<point> vertexes;
};

index_t find_index(point p, std::vector<point>& vertices, double tolerance) {
  for (unsigned i = 0; i < vertices.size(); i++) {
    if (within_eps(p, vertices[i], tolerance)) { return i; }
  }
  vertices.push_back(p);
  return vertices.size() - 1;
}

double min_in_dir(const point n, const std::vector<triangle>& triangles) {
  double min = 1e20;
  for (auto t : triangles) {
    vector<point> verts = {t.v1, t.v2, t.v3};
    double triangle_min = min_distance_along(verts, n);

    if (triangle_min <= min) { min = triangle_min; }
  }

  return min;
}

double max_in_dir(const point n, const std::vector<triangle>& triangles) {
  double max = -1e20;
  for (auto t : triangles) {
    vector<point> verts = {t.v1, t.v2, t.v3};
    double triangle_max = max_distance_along(verts, n);

    if (triangle_max >= max) { max = triangle_max; }
  }

  return max;
}

box bounding_box(const std::vector<triangle>& triangles) {
  double x_min = min_in_dir(point(1, 0, 0), triangles);
  double x_max = max_in_dir(point(1, 0, 0), triangles);

  double y_min = min_in_dir(point(0, 1, 0), triangles);
  double y_max = max_in_dir(point(0, 1, 0), triangles);

  double z_min = min_in_dir(point(0, 0, 1), triangles);
  double z_max = max_in_dir(point(0, 0, 1), triangles);

  return box(x_min, x_max, y_min, y_max, z_min, z_max);
}

std::vector<triangle_t>
fill_vertex_triangles_no_winding_check(const std::vector<triangle>& triangles,
				       std::vector<point>& vertices,
				       double tolerance) {

  box bb = bounding_box(triangles);

  cout << "Bounding box of assembly = " << bb << endl;

  double eps = 0.01;
  double min[3] = {bb.x_min - eps, bb.y_min - eps, bb.z_min - eps};
  double max[3] = {bb.x_max + eps, bb.y_max + eps, bb.z_max + eps};
  double cellsize[3] = {0.1, 0.1, 0.1};
  octree<std::vector<point> > pt_tree(min, max, cellsize);

  cout << "Building octree" << endl;
  for (auto t : triangles) {
    for (auto v : {t.v1, t.v2, t.v3}) {
      double pos[3] = {v.x, v.y, v.z};
      vector<point>& nearby = pt_tree.getCell(pos);
      nearby.push_back(v);
    }
  }

  cout << "Built octree" << endl;

  std::vector<triangle_t> vertex_triangles;
  for (auto t : triangles) {
    auto v1i = find_index(t.v1, vertices, tolerance);
    auto v2i = find_index(t.v2, vertices, tolerance);
    auto v3i = find_index(t.v3, vertices, tolerance);
    triangle_t tr;
    tr.v[0] = v1i;
    tr.v[1] = v2i;
    tr.v[2] = v3i;
    vertex_triangles.push_back(tr);
  }

  return vertex_triangles;
}

// TODO: Add test of agreement for computed normals and
// stored normals in triangles
list_trimesh build_list_trimesh(const std::vector<triangle>& triangles,
				const double tolerance) {
  std::vector<point> vertices;

    

  auto vertex_triangles =
    fill_vertex_triangles_no_winding_check(triangles, vertices, tolerance);

  std::vector<point> face_orientations(triangles.size());
  transform(begin(triangles), end(triangles), begin(face_orientations),
	    [](const triangle t) { return t.normal; });
    
  // delete_duplicate_triangles(vertex_triangles, vertices, face_orientations);    
  // delete_degenerate_triangles(vertex_triangles, vertices, face_orientations);
  // delete_hanging_triangles(vertex_triangles, vertices, face_orientations);

  cout << "# of triangles left = " << vertex_triangles.size() << endl;

  if (vertex_triangles.size() == 0) { return {}; }

  // check_degenerate_triangles(vertex_triangles, vertices);
  // check_non_manifold_triangles(vertex_triangles, vertices);

  // auto initial_comps =
  //   connected_components_by(vertex_triangles, [](const triangle_t l, const triangle_t r)
  // 			   { return share_edge(l, r); });

  // for (vector<unsigned>& comp_inds : initial_comps) {
  //   if (!winding_orders_are_consistent(comp_inds,
  // 					 vertex_triangles,
  // 					 vertices,
  // 					 face_orientations)) {
  // 	for (auto i : comp_inds) {
  // 	  vertex_triangles[i] = flip_winding_order(vertex_triangles[i]);
  // 	}
  //   }
  // }
    
  return list_trimesh{vertex_triangles, vertices};
}


int main(int argc, char *argv[]) {
  DBG_ASSERT(argc == 2);

  string name = argv[1];
  cout << "File Name = " << name << endl;

  arena_allocator a;
  set_system_allocator(&a);

  stl_data data = parse_stl(name);

  cout << "# of triangles = " << data.triangles.size() << endl;

  list_trimesh m = build_list_trimesh(data.triangles, 0.0001);

  cout << "Number of triangles in mesh = " << m.tris.size() << endl;

  vtk_debug_triangles(data.triangles);

}

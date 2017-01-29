#include "geometry/octree.h"
#include "synthesis/visual_debug.h"
#include "system/parse_stl.h"

using namespace gca;

struct list_trimesh {
  std::vector<triangle_t> tris;
  std::vector<point> vertexes;
};

template<typename F>
std::vector<unsigned>
next_connected_component(std::set<unsigned>& face_inds,
			 F neighbors) {

  DBG_ASSERT(face_inds.size() > 0);

  unsigned first = *(face_inds.begin());
  std::deque<unsigned> next;
  next.push_front(first);

  vector<unsigned> comp;

  set<unsigned> already_added{first};

  while (next.size() > 0) {
    unsigned to_use = next.back();
    next.pop_back();
    comp.push_back(to_use);

    DBG_ASSERT(face_inds.find(to_use) != end(face_inds));

    face_inds.erase(to_use);

    for (unsigned i : neighbors(to_use)) {
      if (already_added.find(i) == end(already_added)) {
	next.push_front(i);
	already_added.insert(i);
      }
    }
  }

  std::cout << "Got component of size " << comp.size() << endl;

  return comp;
}

std::vector<std::vector<unsigned> >
connected_components(const list_trimesh& m) {
  std::vector<std::vector<unsigned> > comps;

  cout << "Building trimap" << endl;

  std::unordered_map<index_t, std::vector<unsigned> > trimap;

  for (unsigned j = 0; j < m.tris.size(); j++) {
    const triangle_t& t = m.tris[j];
    for (unsigned i = 0; i < 3; i++) {
      map_insert(trimap, t.v[i], j);
    }
  }

  cout << "Done building trimap" << endl;

  auto neighbors = [&trimap, m](const unsigned& tri) {
    vector<unsigned> nbs;
    const triangle_t& t = m.tris[tri];

    for (unsigned i = 0; i < 3; i++) {
      for (unsigned j : map_find(t.v[i], trimap)) {
	if (j != tri) {
	  nbs.push_back(j);
	}
      }
    }

    nbs = sort_unique(nbs);

    return nbs;
    
  };

  auto finds = inds(m.tris);
  std::set<unsigned> face_inds(begin(finds), end(finds));
  while (face_inds.size() > 0) {
    cout << "Creating component" << endl;
    comps.push_back(next_connected_component(face_inds, neighbors));
  }

  return comps;
}

struct assigned_vertices {
  index_t total_verts;
  std::vector<point> vertices;
  std::unordered_map<const point*, index_t > vertex_map;

  void assign_vertex(const point* p) {
    vertex_map[p] = total_verts;
    vertices.push_back(*p);
    total_verts++;
  }

};

index_t find_index(point p, std::vector<point>& vertices, double tolerance) {
  for (unsigned i = 0; i < vertices.size(); i++) {
    if (within_eps(p, vertices[i], tolerance)) { return i; }
  }
  vertices.push_back(p);
  return vertices.size() - 1;
}

#define EPSILON 0.0001

class nearby_callback : public octree<std::vector<const point*> >::Callback {
public:
  std::vector<const point*> nearby;
  const point* p0;
  double R;

  nearby_callback(const point* p_p0, const double p_R) : p0(p_p0), R(p_R) {}

  virtual bool operator()(const double min[3],
			  const double max[3],
			  std::vector<const point*>& n)
  {
    point pmin(min[0], min[1], min[2]), pmax(max[0], max[1], max[2]);
    double cellSizeSq = ((pmax - pmin).len()) * ((pmax - pmin).len());
    double maxDist = (sqrtf(cellSizeSq) * 0.5f) + R + EPSILON;
        
    point center = 0.5*(pmin + pmax);
    point vectCenter = center - *p0;
    double distCenterSq = vectCenter.len()*vectCenter.len();
    if (distCenterSq > maxDist * maxDist)
      return false;
        
    // Iterate through particles in this cell.
    vector<const point*>::const_iterator it = n.begin();
    for (; it != n.end(); it++)
      {
	const point* p = *it;
	if (p == p0)
	  continue;
	point pp0pos = (*p - *p0); //.lengthsq();
	double dsq = pp0pos.len()*pp0pos.len();
	// If particle is within the radius, increment counter.
	if (dsq <= R * R) {
	  nearby.push_back(p);
	}
      }

    // Subdivide cell if needed.
    return true;
  }
};

index_t find_index(const point* p,
		   assigned_vertices& vertex_map,
		   octree<std::vector<const point*> >& pt_tree,
		   double tolerance) {
  nearby_callback call(p, 2.0*tolerance);

  pt_tree.traverse(&call);

  //cout << "# of nearby points = " << call.nearby.size() << endl;

  for (const point* pt : call.nearby) {
    auto nb = vertex_map.vertex_map.find(pt);
    if (nb != end(vertex_map.vertex_map)) {
      if (within_eps(*(nb->first), *p)) {
	return nb->second;
      }
    }
  }

  vertex_map.assign_vertex(p);

  return vertex_map.total_verts;

  //  DBG_ASSERT(false);
  // for (unsigned i = 0; i < vertices.size(); i++) {
  //   if (within_eps(p, vertices[i], tolerance)) { return i; }
  // }
  // vertices.push_back(p);
  // return vertices.size() - 1;
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
  octree<std::vector<const point*> > pt_tree(min, max, cellsize);

  cout << "Building octree" << endl;

  for (const triangle& t : triangles) {

    for (const point* v : {&(t.v1), &(t.v2), &(t.v3)}) {
      double pos[3] = {v->x, v->y, v->z};
      vector<const point*>& nearby = pt_tree.getCell(pos);
      nearby.push_back(&(t.v1));
    }

  }

  cout << "Built octree" << endl;

  assigned_vertices vertex_map{0, {}, {}};

  std::vector<triangle_t> vertex_triangles;
  for (const triangle& t : triangles) {
    auto v1i = find_index(&(t.v1), vertex_map, pt_tree, tolerance);
    auto v2i = find_index(&(t.v2), vertex_map, pt_tree, tolerance);
    auto v3i = find_index(&(t.v3), vertex_map, pt_tree, tolerance);
    triangle_t tr;
    tr.v[0] = v1i;
    tr.v[1] = v2i;
    tr.v[2] = v3i;
    vertex_triangles.push_back(tr);
  }

  vertices = vertex_map.vertices;

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
    
  cout << "# of triangles left = " << vertex_triangles.size() << endl;

  if (vertex_triangles.size() == 0) { return {}; }

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
  cout << "Number of vertices in mesh  = " << m.vertexes.size() << endl;

  int max_verts = 3*m.tris.size();
  int diff = max_verts - m.vertexes.size();

  cout << "Max possible verts          = " << max_verts << endl;
  cout << "Difference                  = " << diff << endl;

  std::vector<std::vector<unsigned> > ccs =
    connected_components(m);

  cout << "# of connected components = " << ccs.size() << endl;

  unsigned total_inds = 0;
  for (auto& c : ccs) {
    total_inds += c.size();
  }

  cout << "total # of triangles in components =" << total_inds << endl;

  DBG_ASSERT(total_inds = data.triangles.size());

  vtk_debug_triangles(data.triangles);

}

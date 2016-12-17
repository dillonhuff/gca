#include "geometry/triangular_mesh_utils.h"

namespace gca {

  oriented_polygon max_area_outline(const std::vector<index_t>& inds,
				    const triangular_mesh& m) {
    auto part_outlines = mesh_bounds(inds, m);

    DBG_ASSERT(part_outlines.size() > 0);

    oriented_polygon part_outline =
      *(max_element(begin(part_outlines), end(part_outlines),
		    [](const oriented_polygon& l,
		       const oriented_polygon& r)
      { return area(l) < area(r); }));

    return part_outline;
  }

  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh) {
    vector<oriented_polygon> ps;
    if (faces.size() == 0) {
      return ps;
    }
    point normal = mesh.face_orientation(faces.front());
    typedef pair<index_t, index_t> iline;
    vector<iline> tri_lines;
    for (auto i : faces) {
      auto t = mesh.triangle_vertices(i);
      tri_lines.push_back(iline(t.v[0], t.v[1]));
      tri_lines.push_back(iline(t.v[1], t.v[2]));
      tri_lines.push_back(iline(t.v[2], t.v[0]));
    }

    // TODO: Change to sort and count, maybe add to system/algorithm?
    vector<iline> no_ds;
    for (auto l : tri_lines) {
      int count = 0;
      for (auto r : tri_lines) {
	if ((l.first == r.first && l.second == r.second) ||
	    (l.first == r.second && l.second == r.first)) {
	  count++;
	}
      }
      DBG_ASSERT(count > 0);
      if (count == 1) {
	no_ds.push_back(l);
      }
    }

    vector<line> no_dups;
    for (auto l : no_ds) {
      no_dups.push_back(line(mesh.vertex(l.first), mesh.vertex(l.second)));
    }

    return unordered_segments_to_polygons(normal, no_dups);
  }

  vector<polygon_3> surface_boundary_polygons(const vector<index_t>& faces,
					      const triangular_mesh& mesh) {
    auto bounds = mesh_bounds(faces, mesh);

    vector<vector<point> > bound_rings;
    for (auto& bound : bounds) {
      bound_rings.push_back(bound.vertices());
    }

    std::vector<polygon_3> polys = arrange_rings(bound_rings);

    return polys;
  }
  

}

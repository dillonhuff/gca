#include "synthesis/orientation_scheduling.h"
#include "system/algorithm.h"

namespace gca {

  std::vector<point> part_face_orientations(const triangular_mesh& part) {
    auto faces = part.face_indexes();
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).x < part.face_orientation(r).x; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).y < part.face_orientation(r).y; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).z < part.face_orientation(r).z; });
    auto uf = unique(begin(faces), end(faces),
		     [&part](index_t l, index_t r)
		     { return within_eps(part.face_orientation(l), part.face_orientation(r)); });
    faces.erase(uf, end(faces));
    vector<point> orients;
    for (auto f : faces) {
      orients.push_back(part.face_orientation(f));
    }
    return orients;
  }

  std::vector<point>
  rectangular_workpiece_face_orientations(const std::vector<point>& part_faces) {
    std::vector<point> pts = part_faces;
    return part_faces;
  }

  std::vector<point> stable_orientations(const std::vector<point>& faces) {
    return faces;
  }

  point pick_orientation(const std::vector<point>& faces) {
    return faces.front();
  }

  void add_exposed_surfaces(const point orient,
			    const std::vector<point>& part_faces,
			    std::vector<point>& visible_faces) {
  }

  void delete_visible_triangles(std::vector<index_t>& tris,
				const triangular_mesh& mesh,
				const point orient) {
  }

  std::vector<point> workpiece_orientations(const triangular_mesh& part) {
    vector<point> p = part_face_orientations(part);
    auto work_faces = rectangular_workpiece_face_orientations(p);
    auto visible_faces = work_faces;
    //vector<index_t> tri_indexes = part.face_indexes();
    vector<point> orientations;
    while (visible_faces.size() > 0) {
      cout << "# visible faces = " << visible_faces.size() << endl;
      vector<point> s = stable_orientations(visible_faces);
      auto f = pick_orientation(s);
      orientations.push_back(f);
      if (elem(f, work_faces)) {
	add_exposed_surfaces(f, p, visible_faces);
      }
      remove(f, visible_faces);
      //delete_visible_triangles(tri_indexes, part, f);
    }
    return orientations;
  }
}

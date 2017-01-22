#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "process_planning/axis_location.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"
#include "utils/algorithm.h"
#include "system/file.h"
#include "system/parse_stl.h"

using namespace gca;
using namespace std;

std::vector<surface> find_access_surfaces(const std::vector<surface>& surf_complex) {
  DBG_ASSERT(surf_complex.size() > 0);

  const triangular_mesh& m = surf_complex.front().get_parent_mesh();

  vector<surface> access_surfs;

  std::vector<index_t> face_inds;
  for (auto& s : surf_complex) {
    concat(face_inds, s.index_list());
  }
  
  for (auto& s : surf_complex) {
    point s_n = normal(s);
    cout << "Trying normal = " << s_n << endl;

    vector<index_t> vert_or_horiz =
      select(face_inds, [s_n, m](const index_t& i) {
	  return all_parallel_to({i}, m, s_n, 1.0) ||
	  all_orthogonal_to({i}, m, s_n, 1.0);
	});

    if (vert_or_horiz.size() == face_inds.size()) {
      
      auto millable_faces = prismatic_millable_faces(s_n, m);

      if (intersection(millable_faces, face_inds).size() == face_inds.size()) {
	cout << "Viable direction = " << s_n << endl;
	access_surfs.push_back(s);
      }

    }
  }

  return access_surfs;
}

void build_mandatory_complexes(const triangular_mesh& part) {
  auto regions = const_orientation_regions(part);
  vector<surface> const_surfs = inds_to_surfaces(regions, part);
  vector<vector<surface> > surf_complexes =
    connected_components_by_elems(const_surfs,
				  [](const surface& l, const surface& r) {
				    return share_orthogonal_valley_edge(l, r);
				  });

  delete_if(surf_complexes, [](const vector<surface>& surf_complex) {
      return surf_complex.size() < 2;
    });

  cout << "# of valley edge complexes = " << surf_complexes.size() << endl;

  for (auto& s : surf_complexes) {
    vector<surface> access_surfaces =
      find_access_surfaces(s);

    if ((access_surfaces.size() == 0) || (access_surfaces.size() > 2)) {
      cout << "Unmillable part" << endl;
      break;
    }

    for (auto& s : access_surfaces) {
      cout << "Viable normal = " << normal(s) << endl;
    }
    vtk_debug_highlight_inds(s);
  }
}

std::vector<surface>
find_locating_surfaces(const triangular_mesh& part, const double surf_fraction) {
  double sa = part.surface_area();
  std::vector<surface> outer_surfs = outer_surfaces(part);

  delete_if(outer_surfs,
	    [surf_fraction, sa](const surface& s) {
	      return (s.surface_area() / sa) < surf_fraction;
	    });

  return outer_surfs;
}

void surface_plans(const triangular_mesh& part) {
  //build_mandatory_complexes(part);
  vector<surface> locating_surfs = find_locating_surfaces(part, 0.05);

  if (locating_surfs.size() > 0) {
    vtk_debug_highlight_inds(locating_surfs);
  }
}

int main(int argc, char* argv[]) {

  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);
  
  auto check_mesh = [](const std::string& n) {
    cout << "Reading = " << n << endl;

    auto mesh = parse_stl(n, 0.001);

    surface_plans(mesh);

    // box bounding = mesh.bounding_box();

    // cout << "Bounding box = " << endl;
    // cout << bounding << endl;

    // vtk_debug_mesh(mesh);

    // point axis = part_axis(mesh);
    // point neg_axis = part_axis(mesh);

    // cout << "Major part axis   = " << axis << endl;
    // cout << "Length along axis = " << diameter(axis, mesh) << endl;

    // feature_decomposition* f = build_feature_decomposition(mesh, axis);
    // feature_decomposition* g = build_feature_decomposition(mesh, -1*axis);

    // cout << "# of features along " << axis << "  = " << collect_features(f).size() << endl;
    // cout << "# of features along " << neg_axis << " = " << collect_features(g).size() << endl;

    // vtk_debug_mesh(mesh);
    //    vtk_debug_feature_tree(f);
  };

  read_dir(argv[1], check_mesh);

}  

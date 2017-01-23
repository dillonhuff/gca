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

template<typename T>
bool elems_equal(const std::vector<T>& x,
		 const std::vector<T>& y) {
  if (x.size() != y.size()) { return false; }

  for (unsigned i = 0; i < x.size(); i++) {
    if (x[i] != y[i]) { return false; }
  }
  return true;
}

std::vector<surface> find_access_surfaces(const std::vector<surface>& surf_complex) {
  DBG_ASSERT(surf_complex.size() > 0);

  const triangular_mesh& m = surf_complex.front().get_parent_mesh();

  vector<surface> access_surfs;

  std::vector<index_t> face_inds;
  for (auto& s : surf_complex) {
    concat(face_inds, s.index_list());
  }
  face_inds = sort_unique(face_inds);
  
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

      millable_faces = sort_unique(millable_faces);
      if (intersection(millable_faces, face_inds).size() == face_inds.size()) {
      //if (elems_equal(millable_faces, face_inds)) {
	//	cout << "Viable direction = " << s_n << endl;
	access_surfs.push_back(s);
      }

    }
  }

  return access_surfs;
}

struct mandatory_complex {
  vector<point> legal_access_dirs;
  vector<surface> surfs;
};

boost::optional<std::vector<mandatory_complex> >
build_mandatory_complexes(const triangular_mesh& part) {

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

  std::vector<mandatory_complex> complexes;

  for (auto& s : surf_complexes) {
    vector<surface> access_surfaces =
      find_access_surfaces(s);

    if ((access_surfaces.size() == 0) || (access_surfaces.size() > 2)) {
      cout << "Unmillable part" << endl;
      return boost::none;
    }

    vector<point> access_dirs;
    for (auto& s : access_surfaces) {
      cout << "Viable normal = " << normal(s) << endl;
      access_dirs.push_back(normal(s));
    }

    complexes.push_back(mandatory_complex{access_dirs, s});

    //    vtk_debug_highlight_inds(s);
  }

  return complexes;
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

struct proto_setup {
  plane locating_plane;

  vector<vector<surface> > mandatory_complexes;
  vector<surface> mandatory_access;
  vector<surface> unrestricted;

  inline point access_direction() const { return -1*locating_plane.normal(); }
  
};


void visualize_proto_setups(const std::vector<proto_setup>& proto_setups) {
  cout << "# of proto setups = " << proto_setups.size() << endl;

  if (proto_setups.size() == 0) { return; }

  for (auto& ps : proto_setups) {
    cout << "setup access direction = " << ps.access_direction() << endl;

    vector<surface> surfs;
    for (auto& mc : ps.mandatory_complexes) {
      concat(surfs, mc);
    }

    for (auto& c : ps.mandatory_access) {
      surfs.push_back(c);
    }

    for (auto& c : ps.unrestricted) {
      surfs.push_back(c);
    }
    
    if (surfs.size() > 0) {
      vtk_debug_highlight_inds(surfs);
    }
  }
}

boost::optional<std::vector<proto_setup> >
assign_complexes_to_setups(const std::vector<surface>& locating_surfs,
			   const std::vector<mandatory_complex>& mandatory_complexes) {
  std::vector<proto_setup> protos;
  for (auto& s : locating_surfs) {
    bool found_duplicate = false;

    for (auto& ps : protos) {
      if (angle_eps(ps.access_direction(), -1*normal(s), 0.0, 0.5)) {
	found_duplicate = true;
	break;
      }
    }

    if (!found_duplicate) {
      protos.push_back(proto_setup{surface_plane(s), {}, {}, {}});
    }
  }

  for (auto& mc : mandatory_complexes) {

    bool found_setup = false;
    for (proto_setup& ps : protos) {
      if (angle_eps(ps.access_direction(), mc.legal_access_dirs.front(), 0.0, 0.5)) {
	ps.mandatory_complexes.push_back(mc.surfs);
	found_setup = true;
	break;
      }
    }

    if (!found_setup) {
      return boost::none;
    }
  }

  unsigned num_complexes = 0;
  for (auto& ps : protos) {
    cout << "# of mandatory complexes = " << ps.mandatory_complexes.size() << endl;
    num_complexes += ps.mandatory_complexes.size();
  }

  DBG_ASSERT(num_complexes == mandatory_complexes.size());

  return protos;
}

bool
assign_surfaces_to_setups(const triangular_mesh& part,
			  std::vector<proto_setup>& proto_setups) {
  vector<index_t> inds = part.face_indexes();
  for (auto& ps : proto_setups) {
    for (auto& sc : ps.mandatory_complexes) {
      for (auto& s : sc) {
	subtract(inds, s.index_list());
      }
    }
  }

  if (inds.size() == 0) { return true; }

  auto inds_cpy = inds;
  auto const_regions = normal_delta_regions(inds_cpy, part, 1.0);
  std::vector<surface> flat_surfs = inds_to_surfaces(const_regions, part);

  std::unordered_map<surface*, std::vector<proto_setup*> > surf_viz;
  for (auto& ps : proto_setups) {

    auto millable_inds = millable_faces(ps.access_direction(), part);

    for (auto& s : flat_surfs) {
      if (intersection(s.index_list(), millable_inds).size() == s.index_list().size()) {
	map_insert(surf_viz, &s, &ps);
      }
    }
  }

  for (auto& sp : surf_viz) {
    surface* s = sp.first;
    vector<proto_setup*> setups = sp.second;
    if (setups.size() == 0) {
      return false;
    }

    if (setups.size() == 1) {
      (setups.front())->mandatory_access.push_back(*s);
      subtract(inds, s->index_list());
    } else {
      (setups.front())->mandatory_access.push_back(*s);
      subtract(inds, s->index_list());
    }
  }

  if (inds.size() != 0) {
    cout << "Extra inds? " << endl;
    vtk_debug_highlight_inds(inds, part);

    DBG_ASSERT(inds.size() == 0);
  }

  return true;

}

void surface_plans(const triangular_mesh& part) {
  vector<surface> locating_surfs = find_locating_surfaces(part, 0.005);

  if (locating_surfs.size() == 0) {
    cout << "Unmillable: No locating surfaces" << endl;
    return;
  }

  auto mandatory_complexes = build_mandatory_complexes(part);

  if (!mandatory_complexes) {
    cout << "Unmillable: Unreachable surface complex" << endl;
    return;
  }

  boost::optional<std::vector<proto_setup> > proto_setups =
    assign_complexes_to_setups(locating_surfs, *mandatory_complexes);


  if (!proto_setups) {
    cout << "Unmillable: Mandatory setup that does not have a locating surface" << endl;
    return;
  }

  bool res = assign_surfaces_to_setups(part, *proto_setups);

  if (!res) {
    cout << "Unmillable: Surface cannot be assigned to a setup" << endl;
    return;
  }
  
  cout << "MILLABLE!" << endl;

  visualize_proto_setups(*proto_setups);
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

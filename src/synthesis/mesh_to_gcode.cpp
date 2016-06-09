#include "geometry/matrix.h"
#include "synthesis/axis_3.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/millability.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

  typedef std::vector<std::vector<index_t>> surface_list;

  ostream& operator<<(ostream& out, const workpiece& w) {
    out << "WORKPIECE" << endl;
    out << w.sides[0] << endl;
    out << w.sides[1] << endl;
    out << w.sides[2] << endl;
    return out;
  }

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh) {
    vector<point> normals;
    normals.push_back(point(1, 0, 0));
    normals.push_back(point(-1, 0, 0));
    normals.push_back(point(0, 1, 0));
    normals.push_back(point(0, -1, 0));
    normals.push_back(point(0, 0, 1));
    normals.push_back(point(0, 0, -1));

    for (auto& sf : part_surfaces) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  sf.set_SA();
	}
      }
    }
  }

  // TODO: Change to actually align instead of just making surfaces
  // orthogonal to axes
  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w) {
    return w;
  }

  bool any_SA_surface_contains(index_t i,
			       const std::vector<surface>& surfaces) {
    for (auto surface : surfaces) {
      if (surface.is_SA() && surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices) {
    delete_if(indices,
	      [&surfaces](index_t i)
	      { return any_SA_surface_contains(i, surfaces); });
  }

  void remove_SA_surfaces(const std::vector<surface>& stable_surfaces,
			  std::vector<surface>& cut_surfaces) {
    delete_if(cut_surfaces,
    	      [&stable_surfaces](const surface& s) {
		for (auto other : stable_surfaces) {
		  if (s.contained_by(other)) {
		    return true;
		  }
		}
		return false;
	      });
  }
  
  std::vector<surface> outer_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    vector<surface> surfaces;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      if (is_outer_surface(f, part)) {
	surfaces.push_back(surface(&part, f));
      }
    }
    return surfaces;
  }

  bool orthogonal_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 90, 0.1);
  }

  bool parallel_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 180, 0.1);
  }
  
  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces) {
    vector<stock_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = &(surfaces[j]);
      for (unsigned k = 0; k < surfaces.size(); k++) {
	const surface* next_right = &(surfaces[k]);
	if (parallel_flat_surfaces(next_right, next_left)) {
	  for (unsigned l = 0; l < surfaces.size(); l++) {
	    const surface* next_bottom = &(surfaces[l]);
	    if (orthogonal_flat_surfaces(next_bottom, next_left)) {
	      orients.push_back(stock_orientation(next_left,
						  next_right,
						  next_bottom));
	    }
	  }
	}
      }
    }
    assert(orients.size() > 0);
    return orients;
  }

  std::vector<unsigned>
  surfaces_millable_from(const stock_orientation& orient,
			 const std::vector<surface>& surfaces_left) {
    std::vector<index_t> millable =
      millable_faces(orient.top_normal(), orient.get_mesh());
    sort(begin(millable), end(millable));
    vector<unsigned> mill_surfaces;
    for (unsigned i = 0; i < surfaces_left.size(); i++) {
      if (surfaces_left[i].contained_by_sorted(millable)) {
	mill_surfaces.push_back(i);
      }
    }
    return mill_surfaces;
  }
  
  std::vector<surface>
  cut_surfaces(const triangular_mesh& part) {
    double normal_degrees_delta = 30.0;
    auto inds = part.face_indexes();
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, part, normal_degrees_delta);
    vector<surface> surfaces;
    for (auto r : delta_regions) {
      surfaces.push_back(surface(&part, r));
    }
    return surfaces;
  }

  typedef std::map<unsigned, std::vector<unsigned>> orientation_map;

  orientation_map
  greedy_possible_orientations(const std::vector<surface>& surfaces,
			       std::vector<stock_orientation>& all_orients) {
    // TODO: Better way to do this?
    sort(begin(all_orients), end(all_orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().x < r.top_normal().x; });
    sort(begin(all_orients), end(all_orients),
    	 [](const stock_orientation l, const stock_orientation r)
    	 { return l.top_normal().z < r.top_normal().z; });

    orientation_map orient_map;
    vector<unsigned> surfaces_left = inds(surfaces);
    vector<unsigned> orients_left = inds(all_orients);
    while (surfaces_left.size() > 0) {
      assert(orients_left.size() > 0);
      unsigned next_orient = orients_left.back();
      orients_left.pop_back();
      auto surfaces_cut = surfaces_millable_from(all_orients[next_orient], surfaces);
      subtract(surfaces_left, surfaces_cut);
      for (auto surface_ind : surfaces_cut) {
	if (orient_map.find(surface_ind) != end(orient_map)) {
	  auto k = orient_map.find(surface_ind);
	  auto orients = k->second;
	  orients.push_back(next_orient);
	  orient_map[surface_ind] = orients;
	} else {
	  vector<unsigned> orients{next_orient};
	  orient_map[surface_ind] = orients;
	}
      }
    }
    return orient_map;
  }

  bool connected_by(const unsigned i,
  		    const unsigned j,
  		    const std::vector<surface>& surfaces,
  		    const triangular_mesh& part,
		    const unsigned orient_ind,
  		    const orientation_map& orient_map) {
    auto ind1 = surfaces[i].index_list();
    auto ind2 = surfaces[j].index_list();
    if (share_edge(ind1, ind2, part)) {
      return elem(orient_ind, orient_map.find(i)->second) &&
	elem(orient_ind, orient_map.find(j)->second);
    }
    return false;
  }

  // TODO: Is there a way to do this without const?
  std::vector<unsigned>
  select_surfaces(unsigned orient_ind,
		  std::vector<unsigned>& surfaces_left,
		  const std::vector<surface>& surfaces_to_cut,
		  const orientation_map& possible_orientations,
		  const triangular_mesh& part) {
    vector<unsigned> initial_surfaces;
    for (auto i : surfaces_left) {
      vector<unsigned> viable_orients = possible_orientations.find(i)->second;
      if (elem(orient_ind, viable_orients)) {
	initial_surfaces.push_back(i);
      }
    }
    vector<unsigned> surfaces_cut;
    for (auto s_ind : initial_surfaces) {
      vector<unsigned> rest =
  	greedy_chain(s_ind,
		     surfaces_left,
  		     [orient_ind, part, possible_orientations, surfaces_to_cut]
		     (const unsigned i, const unsigned j) {
  		       return connected_by(i, j, surfaces_to_cut, part, orient_ind, possible_orientations);
  		     });
      subtract(surfaces_left, rest);
      concat(surfaces_cut, rest);
    }
    return surfaces_cut;
  }

  std::vector<std::pair<stock_orientation, surface_list>>
    greedy_pick_orientations(const std::vector<surface>& surfaces_to_cut,
			     const std::vector<stock_orientation>& all_orients,
  			     orientation_map& possible_orientations,
			     const triangular_mesh& part) {
    vector<unsigned> surfaces_left = inds(surfaces_to_cut);
    vector<unsigned> orientations_left = inds(all_orients);
    vector<pair<stock_orientation, surface_list>> orients;
    for (auto orient : possible_orientations) {
      if (surfaces_left.size() == 0) { return orients; }

      vector<unsigned> orients_to_choose_from = orient.second;
      auto inter_orients = intersection(orients_to_choose_from,
					orientations_left);

      if (inter_orients.size() > 0) {
	unsigned orient_ind = inter_orients.back();
	cout << "Trying orientation " << orient_ind << " with top normal ";
	cout << all_orients[orient_ind].top_normal() << endl;
	remove(orient_ind, orientations_left);
	vector<unsigned> surfaces_cut =
	  select_surfaces(orient_ind, surfaces_left, surfaces_to_cut, possible_orientations, part);
	cout << "Selected " << surfaces_cut.size() << " surfaces" << endl;
	surface_list surfaces;
	if (surfaces_cut.size() > 0) {
	  for (auto i : surfaces_cut) {
	    surfaces.push_back(surfaces_to_cut[i].index_list());
	  }
	  orients.push_back(mk_pair(all_orients[orient_ind], surfaces));
	}
	subtract(surfaces_left, surfaces_cut);
      }
    }

    assert(surfaces_left.size() == 0);
    return orients;
  }
  
  std::vector<std::pair<stock_orientation, surface_list>>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& stable_surfaces) {
    vector<stock_orientation> all_orients =
      all_stable_orientations(stable_surfaces);

    vector<surface> surfaces_to_cut = cut_surfaces(part_mesh);
    cout << "# initial faces = " << surfaces_to_cut.size() << endl;
    remove_SA_surfaces(stable_surfaces, surfaces_to_cut);
    cout << "# faces left = " << surfaces_to_cut.size() << endl;

    orientation_map possible_orientations =
      greedy_possible_orientations(surfaces_to_cut, all_orients);
    assert(surfaces_to_cut.size() == 0 || possible_orientations.size() > 0);

    cout << "ORIENTATION MAP: " << endl;
    for (auto m : possible_orientations) {
      cout << "Surface " << m.first << " can be cut from orientations" << endl;
      for (auto orient_ind : m.second) {
	cout << orient_ind << " with top normal " << all_orients[orient_ind].top_normal() << endl;
      }
    }

    return greedy_pick_orientations(surfaces_to_cut,
				    all_orients,
				    possible_orientations,
				    part_mesh);
  }

  triangular_mesh orient_mesh(const triangular_mesh& mesh,
			      const stock_orientation& orient) {
    point normal = orient.top_normal();
    matrix<3, 3> top_rotation_mat = rotate_onto(normal, point(0, 0, 1));
    auto m = top_rotation_mat * mesh;
    return m;
  }

  triangular_mesh shift_mesh(const triangular_mesh& mesh,
			     const vice v) {
    double x_f = v.x_max();
    double y_f = v.fixed_clamp_y();
    double z_f = v.base_z();
    point shift(x_f - max_in_dir(mesh, point(1, 0, 0)),
		y_f - max_in_dir(mesh, point(0, 1, 0)),
		z_f - min_in_dir(mesh, point(0, 0, 1)));
    auto m = mesh.apply_to_vertices([shift](const point p)
		      { return p + point(shift.x, shift.y, shift.z); });
    return m;
  }

  std::pair<triangular_mesh, std::vector<std::vector<index_t>>>
  oriented_part_mesh(const stock_orientation& orient,
		     const surface_list& surfaces,
		     const vice v) {
    auto mesh = orient.get_mesh();
    auto oriented_mesh = orient_mesh(mesh, orient);
    return mk_pair(shift_mesh(oriented_mesh, v), surfaces);
  }

  gcode_program cut_secured_mesh(const std::pair<triangular_mesh, std::vector<std::vector<index_t>>>& mesh_surfaces_pair,
				 const vice v,
				 const std::vector<tool>& tools) {
    tool t = *(min_element(begin(tools), end(tools),
			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    double cut_depth = 0.2;
    double h = max_in_dir(mesh_surfaces_pair.first, point(0, 0, 1));
    vector<polyline> lines = mill_surfaces(mesh_surfaces_pair.second,
					   mesh_surfaces_pair.first,
					   t,
					   cut_depth,
					   h);
    double safe_z = h + t.length() + 0.1;
    return gcode_program("Surface cut", emco_f1_code(lines, safe_z));
  }

  void cut_secured_meshes(const std::vector<std::pair<triangular_mesh, surface_list>>& meshes,
			  std::vector<gcode_program>& progs,
			  const vice v,
			  const std::vector<tool>& tools) {
    for (auto mesh_surface_pair : meshes) {
      progs.push_back(cut_secured_mesh(mesh_surface_pair, v, tools));
    }
  }

  std::vector<std::pair<triangular_mesh, surface_list>>
  part_arrangements(const triangular_mesh& part_mesh,
		    const vector<surface>& part_ss,
		    const vice v) {
    vector<pair<stock_orientation, surface_list>> orients =
      orientations_to_cut(part_mesh, part_ss);
    vector<pair<triangular_mesh, surface_list>> meshes;
    for (auto orient_surfaces_pair : orients) {
      cout << "Top normal " << orient_surfaces_pair.first.top_normal() << endl;
      meshes.push_back(oriented_part_mesh(orient_surfaces_pair.first,
					  orient_surfaces_pair.second,
					  v));
    }
    return meshes;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece w) {
    auto part_ss = outer_surfaces(part_mesh);
    auto aligned_workpiece = align_workpiece(part_ss, w);
    classify_part_surfaces(part_ss, aligned_workpiece);
    vector<pair<triangular_mesh, surface_list>> meshes =
      part_arrangements(part_mesh, part_ss, v);
    vector<gcode_program> ps =
      workpiece_clipping_programs(aligned_workpiece, part_mesh, tools, v);
    cut_secured_meshes(meshes, ps, v, tools);
    return ps;
  }

}

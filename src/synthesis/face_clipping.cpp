#include "synthesis/face_clipping.h"

namespace gca {

  workpiece clipped_workpiece(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh) {
    point x_n = aligned_workpiece.sides[0].normalize();
    point y_n = aligned_workpiece.sides[1].normalize();
    point z_n = aligned_workpiece.sides[2].normalize();
    
    point x_d = diameter(aligned_workpiece.sides[0], part_mesh) * x_n;
    point y_d = diameter(aligned_workpiece.sides[1], part_mesh) * y_n;
    point z_d = diameter(aligned_workpiece.sides[2], part_mesh) * z_n;

    return workpiece(x_d, y_d, z_d, aligned_workpiece.stock_material);
  }
  
  // TODO: Correctly place boxes in space relative to the vices
  std::pair<fixture_setup, fixture_setup>
  clip_axis(double workpiece_x,
	    double workpiece_y,
	    double workpiece_height,
	    double eps,
	    double part_height,
	    const vice v) {
    assert(workpiece_height > part_height);

    double z_max = workpiece_height + 0.01;

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, z_max - 5.0, z_max);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    
    box b = box(0, workpiece_x,
		0, workpiece_y,
		z_max - eps, z_max);

    pocket p = box_pocket(b);
    vector<pocket> ps{p};

    box b2 = box(0, workpiece_x,
		 0, workpiece_y,
		 part_height, z_max);
    pocket q = box_pocket(b2);
    vector<pocket> qs{q};

    return std::make_pair(fixture_setup(m, v, ps), fixture_setup(m, v, qs));
  }

  void
  append_clip_setups(const string& axis,
		       const int axis_number,
		       const workpiece aligned_workpiece,
		       const workpiece clipped,
		       const double eps,
		       const vice v,
		       std::vector<fixture_setup>& clip_progs) {
    double a1 = aligned_workpiece.sides[(axis_number + 1) % 3].len();
    double a2 = aligned_workpiece.sides[(axis_number + 2) % 3].len();

    double workpiece_x = max(a1, a2);
    double workpiece_y = min(a1, a2);

    double workpiece_height = aligned_workpiece.sides[axis_number].len() + v.base_z();
    double part_height = clipped.sides[axis_number].len() + v.base_z();

    auto clip_x = clip_axis(workpiece_x,
    			    workpiece_y,
    			    workpiece_height,
    			    eps,
    			    part_height,
    			    v);

    clip_progs.push_back(clip_x.first);
    clip_progs.push_back(clip_x.second);
  }

  std::vector<fixture_setup>
  axis_by_axis_clipping(std::vector<surface>& surfaces_to_cut,
			const workpiece& aligned_workpiece,
			const workpiece& clipped,
			const fixtures& f) {
    triangular_mesh wp_mesh = stock_mesh(aligned_workpiece);

    vector<fixture_setup> clip_setups;
    double eps = 0.05;
    append_clip_setups("X", 0, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    append_clip_setups("Y", 1, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    append_clip_setups("Z", 2, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    return clip_setups;
  }

  clamp_orientation
  next_orthogonal_to_all(const std::vector<clamp_orientation>& to_check,
			 const std::vector<clamp_orientation>& to_return_from) {
    assert(to_return_from.size() > 0);
      
    if (to_check.size() == 0) { return to_return_from.front(); }

    auto orthogonal_to = [](const point l, const point r)
      { return within_eps(l.dot(r), 0, 0.001); };

    for (unsigned i = 0; i < to_return_from.size(); i++) {
      point q = to_return_from[i].top_normal();
      if (all_of(begin(to_check), end(to_check),
		 [q, orthogonal_to](const clamp_orientation& r) { return orthogonal_to(q, r.top_normal()); })) {
	return to_return_from[i];
      }
    }

    assert(false);
  }
  std::vector<clamp_orientation>
  three_orthogonal_orients(const std::vector<clamp_orientation>& orients) {
    assert(orients.size() >= 3);
    vector<clamp_orientation> ortho_orients;
    ortho_orients.push_back(next_orthogonal_to_all(ortho_orients, orients));
    cout << "Got one" << endl;
    ortho_orients.push_back(next_orthogonal_to_all(ortho_orients, orients));
    cout << "Got two" << endl;
    ortho_orients.push_back(next_orthogonal_to_all(ortho_orients, orients));
    cout << "Got three" << endl;
    
    assert(ortho_orients.size() == 3);
    return ortho_orients;
  }

  std::pair<fixture_setup, fixture_setup>
  axis_clip(const clamp_orientation& orient,
	    const std::vector<clamp_orientation>& clamp_orients,
	    const triangular_mesh& aligned_workpiece,
	    const triangular_mesh& part_mesh,
	    const vice& v) {
    auto pos_t = mating_transform(aligned_workpiece, orient, v);
    auto f1 = clip_base(apply(pos_t, aligned_workpiece), apply(pos_t, part_mesh), v);
    
    point neg_orient_dir = -1*orient.top_normal();
    clamp_orientation neg_orient =
      find_orientation_by_normal(clamp_orients, neg_orient_dir);
    auto neg_t = mating_transform(aligned_workpiece, neg_orient, v);
    auto f2 = clip_base(apply(neg_t, aligned_workpiece), apply(neg_t, part_mesh), v);

    return make_pair(f1, f2);
  }

  std::vector<fixture_setup>
  axis_clipping(const triangular_mesh& aligned_workpiece,
		const triangular_mesh& part_mesh,
		const fixtures& f) {
    auto surfs = outer_surfaces(aligned_workpiece);
    assert(surfs.size() == 6);
    vector<clamp_orientation> clamp_orients =
      all_stable_orientations(surfs, f.get_vice());
    cout << "# of orientations = " << clamp_orients.size() << endl;
    assert(clamp_orients.size() >= 6);
    vector<clamp_orientation> axis_orients =
      three_orthogonal_orients(clamp_orients);
    vector<fixture_setup> cut_setups;
    for (auto orient : axis_orients) {
      auto setup_pair = axis_clip(orient, clamp_orients, aligned_workpiece, part_mesh, f.get_vice());
      cut_setups.push_back(setup_pair.first);
      cut_setups.push_back(setup_pair.second);
    }
    return cut_setups;
  }

  clipping_plan
  axis_by_axis_clipping(const workpiece w, 
			const triangular_mesh& part_mesh,
			std::vector<surface>& surfaces_to_cut,
			const std::vector<tool>& tools,
			const fixtures& f) {
    triangular_mesh wp_mesh = align_workpiece(surfaces_to_cut, w);

    auto clip_setups = axis_clipping(wp_mesh, part_mesh, f);

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, wp_mesh);
    remove_contained_surfaces(clipped_surfs, surfaces_to_cut);

    return clipping_plan(clipped_surfs, clip_setups);
  }

}

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

  clipping_plan
  axis_by_axis_clipping(const workpiece w, 
			const triangular_mesh& part_mesh,
			std::vector<surface>& surfaces_to_cut,
			const std::vector<tool>& tools,
			const fixtures& f) {
    auto aligned_workpiece = w;
    triangular_mesh wp_mesh = stock_mesh(aligned_workpiece);

    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);

    auto clip_setups = axis_by_axis_clipping(surfaces_to_cut, aligned_workpiece, clipped, f);

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, wp_mesh);
    remove_contained_surfaces(clipped_surfs, surfaces_to_cut);

    return clipping_plan(clipped_surfs, clip_setups);
  }

}

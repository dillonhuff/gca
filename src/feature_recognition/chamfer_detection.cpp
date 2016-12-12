#include "feature_recognition/chamfer_detection.h"
#include "synthesis/millability.h"

namespace gca {

  bool
  all_constant_angle_wrt(const std::vector<index_t>& faces,
			 const triangular_mesh& mesh,
			 const point n,
			 const double tol) {
    if (faces.size() == 0 || faces.size() == 1) { return true; }

    double first_angle =
      angle_between(mesh.face_orientation(faces.front()), n);
    
    for (auto i : faces) {
      double next_angle =
	angle_between(mesh.face_orientation(i), n);

      if(!within_eps(first_angle, next_angle, 1.0)) {
	return false;
      }
    }

    double chamf_angle = 90 - first_angle;
    cout << "CHAMFER ANGLE = " << chamf_angle << endl;

    return true;
  }

  std::vector<chamfer>
  chamfer_regions(const std::vector<index_t>& non_prismatic_faces,
		  const triangular_mesh& mesh,
		  const point n,
		  const std::vector<tool>& tools) {

    auto npf = non_prismatic_faces;

    vector<vector<index_t>> possible_regions =
      normal_delta_regions(npf, mesh, 180.0);

    delete_if(possible_regions, [mesh, n](const vector<index_t>& inds) {
	return !all_constant_angle_wrt(inds, mesh, n, 1.0);
      });

    vector<chamfer> chamfers;
    for (auto& region : possible_regions) {
      if (region.size() == 0) { continue; }

      double angle_wrt_n =
	90 - angle_between(mesh.face_orientation(region.front()), n);

      for (auto t : tools) {
	if (t.type() == CHAMFER) {
	  if (within_eps(angle_wrt_n, t.angle_per_side_from_centerline(), 1.0)) {
	    chamfers.push_back(chamfer{region, t});
	  }
	}
      }

    }

    return chamfers;

    // delete_if(possible_regions,
    // 	      [mesh, chamfer_angles, n](const vector<index_t>& inds) {
    // 		if (inds.size() == 0) { return true; }

    // 		double angle_wrt_n =
    // 		  90 - angle_between(mesh.face_orientation(inds.front()), n);

    // 		for (auto& t : tools) {
    // 		  if (t.type() == CHAMFER)
    // 		  if (within_eps(angle_wrt_n, angle, 1.0)) {
    // 		    return false;
    // 		  }
    // 		}
    // 		return true;
    // 	      });

    // return possible_regions;
  }

  std::vector<chamfer>
  chamfer_regions(const triangular_mesh& mesh,
		  const point n,
		  const std::vector<tool>& tools) {
    vector<index_t> non_prismatic_faces =
      non_prismatic_millable_faces(n, mesh);

    return chamfer_regions(non_prismatic_faces, mesh, n, tools);
  }

  std::vector<index_t>
  chamfer_faces(const triangular_mesh& mesh,
		const point n,
		const std::vector<tool>& tools) {
    auto chamfers = chamfer_regions(mesh, n, tools);

    vector<index_t> inds;
    for (auto& chamfer : chamfers) {
      concat(inds, chamfer.faces);
    }

    return inds;
  }

}

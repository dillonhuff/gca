#include "feature_recognition/chamfer_detection.h"
#include "synthesis/millability.h"

namespace gca {

  std::vector<index_t>
  non_prismatic_millable_faces(const point n,
			       const triangular_mesh& part) {
    vector<index_t> all_millable_faces = millable_faces(n, part);

    // Just use parallel to or orthogonal_to functions?
    vector<index_t> not_vert_or_horiz =
      select(all_millable_faces, [n, part](const index_t& s) {
	  return !(all_parallel_to({s}, part, n, 1.0) ||
		   all_orthogonal_to({s}, part, n, 1.0));
	});

    return not_vert_or_horiz;
  }


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

    cout << "CHAMFER ANGLE = " << first_angle << endl;

    return true;
  }

  std::vector<std::vector<index_t> >
  chamfer_regions(const std::vector<index_t>& non_prismatic_faces,
		  const triangular_mesh& mesh,
		  const point n,
		  const std::vector<double>& chamfer_angles) {

    auto npf = non_prismatic_faces;

    vector<vector<index_t>> possible_regions =
      normal_delta_regions(npf, mesh, 180.0);

    delete_if(possible_regions, [mesh, n](const vector<index_t>& inds) {
	return !all_constant_angle_wrt(inds, mesh, n, 1.0);
      });

    delete_if(possible_regions,
	      [mesh, chamfer_angles, n](const vector<index_t>& inds) {
		if (inds.size() == 0) { return true; }

		double angle_wrt_n =
		  angle_between(mesh.face_orientation(inds.front()), n);

		for (auto angle : chamfer_angles) {
		  if (within_eps(angle_wrt_n, angle, 1.0)) {
		    return false;
		  }
		}
		return true;
	      });

    return possible_regions;
  }

  std::vector<std::vector<index_t> >
  chamfer_regions(const triangular_mesh& mesh,
		  const point n,
		  const std::vector<double>& chamfer_angles) {
    vector<index_t> non_prismatic_faces =
      non_prismatic_millable_faces(n, mesh);

    return chamfer_regions(non_prismatic_faces, mesh, n, chamfer_angles);

  }

  std::vector<index_t>
  chamfer_faces(const triangular_mesh& mesh,
		const point n,
		const std::vector<double>& chamfer_angles) {
    auto chamfers = chamfer_regions(mesh, n, chamfer_angles);

    vector<index_t> inds;
    for (auto& chamfer_faces : chamfers) {
      concat(inds, chamfer_faces);
    }

    return inds;
  }

}

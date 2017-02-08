#include "process_planning/major_axis_fixturing.h"

namespace gca {

  boost::optional<dir_fixture>
  find_dir_fixture(const fixtures& f,
		   const point n,
		   const triangular_mesh& m) {
    Nef_polyhedron part_nef = trimesh_to_nef_polyhedron(m);

    vice v = f.get_vice();
    if (f.parallel_plates().size() > 0) {
      plate_height ph = f.parallel_plates().front();

      cout << "Selected parallel plate height = " << ph << endl;
      v = vice(f.get_vice(), ph);
    }

    vector<pair<clamp_orientation, homogeneous_transform>> orients =
      all_stable_orientations_with_side_transforms(part_nef, v, n);

    cout << "# of orients in " << n << " = " << orients.size() << endl;

    if (orients.size() > 0) {
      auto orient =
	max_e(orients,
	      [m]
	      (const std::pair<clamp_orientation, homogeneous_transform>& c)
	      { return c.first.contact_area(m); });

      cout << "Found fixture in " << n << endl;

      homogeneous_transform t = orient.second;

      return dir_fixture{orient.first, t, v};
    }

    return boost::none;
    
  }

  const triangular_mesh& mesh(const major_axis_decomp& d) {
    return d.decomp.positive.front().get_parent_mesh();
  }

  axis_fixture build_axis_fixture(const fixtures& f,
				  const major_axis_decomp& d) {
    const triangular_mesh& m = mesh(d);

    boost::optional<dir_fixture> positive =
      find_dir_fixture(f, d.major_axis, m);

    boost::optional<dir_fixture> negative =
      find_dir_fixture(f, -1*(d.major_axis), m);

    return axis_fixture{positive, negative};
  }

}

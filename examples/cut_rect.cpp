#include "synthesis/timing.h"
#include "backend/freeform_toolpaths.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/freeform_surface_detection.h"
#include "synthesis/clamp_orientation.h"
#include "backend/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/visual_debug.h"
#include "backend/toolpath_generation.h"
#include "system/parse_stl.h"

using namespace gca;

std::vector<std::vector<cut*>>
shift_cuts(const point s,
	   const std::vector<std::vector<cut*>>& cuts_list) {
  vector<vector<cut*>> shifted_cut_list;
  for (auto cuts : cuts_list) {
    vector<cut*> shifted_cuts;
    for (auto c : cuts) {
      shifted_cuts.push_back(c->shift(s));
    }
    shifted_cut_list.push_back(shifted_cuts);
  }
  return shifted_cut_list;
}

toolpath shift(const point s, const toolpath& tp) {
  toolpath shifted_toolpath = tp;
  std::vector<std::vector<cut*>> shifted_lines =
	      shift_cuts(s, shifted_toolpath.cuts_without_safe_moves());

  double shifted_safe_tlc =
    shifted_toolpath.safe_z_before_tlc + s.z;

  return toolpath(tp.pocket_type(),
		  shifted_safe_tlc,
		  tp.spindle_speed,
		  tp.feedrate,
		  tp.plunge_feedrate,
		  tp.t,
		  shifted_lines);
}

std::vector<toolpath> shift(const point s,
			    const std::vector<toolpath>& toolpaths) {
  vector<toolpath> shifted_toolpaths;
  for (auto& toolpath : toolpaths) {
    shifted_toolpaths.push_back(shift(s, toolpath));
  }
  return shifted_toolpaths;
}

rigid_arrangement shift(const point s, const rigid_arrangement& a) {
  rigid_arrangement shifted_a;

  for (auto n : a.mesh_names()) {
    shifted_a.insert(n, shift(s, a.mesh(n)));
    shifted_a.set_metadata(n, a.metadata(n));

  }

  return shifted_a;

}

fabrication_setup shift(const point s,
			const fabrication_setup& setup) {
  rigid_arrangement shifted_setup = shift(s, setup.arrangement());
  vice shifted_vice = shift(s, setup.v);
  vector<toolpath> shifted_toolpaths = shift(s, setup.toolpaths());

  return fabrication_setup(shifted_setup, shifted_vice, shifted_toolpaths);
}

void print_setup_info(const fabrication_setup& shifted_setup) {
  if (shifted_setup.v.has_parallel_plate()) {
    cout << "Vice uses parallel plate of height " << shifted_setup.v.plate_height() << endl;
  } else {
    cout << "No parallel plate" << endl;
  }

  double part_len_x = diameter(point(1, 0, 0), shifted_setup.part_mesh());
  double part_len_y = diameter(point(0, 1, 0), shifted_setup.part_mesh());
  double part_len_z = diameter(point(0, 0, 1), shifted_setup.part_mesh());

  cout << "Part length along x axis in setup = " << part_len_x << endl;
  cout << "Part length along y axis in setup = " << part_len_y << endl;
  cout << "Part length along z axis in setup = " << part_len_z << endl;
}

int main(int argc, char *argv[]) {

  DBG_ASSERT(argc == 2);

  string name = argv[1];
  cout << "File Name = " << name << endl;

  arena_allocator a;
  set_system_allocator(&a);

  triangular_mesh mesh =
    parse_stl(name, 0.0001);

  vtk_debug_mesh(mesh);

  point n(0, 0, 1);

  vice test_v = current_setup(); //custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
  vice test_vice = top_jaw_origin_vice(test_v);
    
  std::vector<plate_height> parallel_plates{0.4995};
  fixtures fixes(test_vice, parallel_plates);

  tool t1{1.0 / 8.0, 3.94, 4, HSS, BALL_NOSE};
  t1.set_cut_diameter(0.28);
  t1.set_cut_length(1.25);

  t1.set_shank_diameter(0.5);
  t1.set_shank_length(0.05);

  t1.set_holder_diameter(2.5);
  t1.set_holder_length(3.5);
  t1.set_tool_number(1);
    
  tool t2{1.0, 3.94, 4, HSS, FLAT_NOSE};
  t2.set_cut_diameter(0.1);
  t2.set_cut_length(1.8);

  t2.set_shank_diameter(1.1);
  t2.set_shank_length(0.05);

  t2.set_holder_diameter(2.5);
  t2.set_holder_length(3.5);

  vector<tool> tools{t1, t2};

  auto outer_surfs = outer_surfaces(mesh);
  auto orients = all_stable_orientations(outer_surfs, test_v);

  auto orient = find_orientation_by_normal(orients, n);
  fixture f(orient, test_v);
  auto t = balanced_mating_transform(mesh, orient, test_v);

  mesh = apply(t, mesh);

  auto surfs = freeform_surface_regions(mesh, n, tools);
  vector<pocket> pockets;
  for (auto& freeform : surfs) {
    pockets.push_back(freeform_operation(freeform.s, freeform.tools));
  }
  
  auto setup = fixture_setup(&mesh, f, pockets);

  auto toolpaths = mill_pockets(pockets, ALUMINUM);

  fabrication_setup step(mesh, test_v, toolpaths);
  // auto aligned = apply(t, wp_mesh);

  cout << "Programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  point zero_pos = gui_select_part_zero(step);
  cout << "Part zero position = " << zero_pos << endl;

  fabrication_setup shifted_setup = shift(-1*zero_pos, step);
  print_setup_info(shifted_setup);
  visual_debug(shifted_setup);

  print_setup_info(shifted_setup);
  cout << "Program for setup" << endl;
  auto program = shifted_setup.gcode_for_toolpaths(emco_f1_code_no_TLC);
  cout << program.name << endl;
  cout << program.blocks << endl;

  visual_debug(step);
  
  return EXIT_SUCCESS;
}

#include "backend/timing.h"
#include "geometry/vtk_utils.h"
#include "synthesis/clamp_orientation.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/visual_debug.h"
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

fabrication_inputs part_1_2_inputs() {
  vice test_v = current_setup();
  vice test_vice = top_jaw_origin_vice(test_v);

  std::vector<plate_height> plates{0.48}; //0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(1.5, 1.5, 1.5, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.14);
  t1.set_cut_length(0.5);

  t1.set_shank_diameter(.375); //3.0 / 8.0);
  t1.set_shank_length(0.18);

  t1.set_holder_diameter(1.8);
  t1.set_holder_length(3.0);
  t1.set_tool_number(1);

  tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.335);
  t2.set_cut_length(0.72);

  t2.set_shank_diameter(0.336);
  t2.set_shank_length(0.01);

  t2.set_holder_diameter(1.8);
  t2.set_holder_length(3.0);
  t2.set_tool_number(2);

  tool t3(0.5, 3.0, 2, HSS, FLAT_NOSE);
  t3.set_cut_diameter(0.5);
  t3.set_cut_length(0.7);

  t3.set_shank_diameter(0.7);
  t3.set_shank_length(0.5);

  t3.set_holder_diameter(1.8);
  t3.set_holder_length(3.0);
  t3.set_tool_number(3);
  
  vector<tool> tools{t1, t2, t3}; //, t3, t4};

  return fabrication_inputs(fixes, tools, workpiece_dims);
}

fabrication_inputs octagon_with_holes_short_inputs() {
  vice test_v = current_setup();
  vice test_vice = top_jaw_origin_vice(test_v);

  std::vector<plate_height> plates{0.48}; //0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(1.5, 1.58, 1.5, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.14);
  t1.set_cut_length(0.5);

  t1.set_shank_diameter(.375); //3.0 / 8.0);
  t1.set_shank_length(0.18);

  t1.set_holder_diameter(1.8);
  t1.set_holder_length(3.0);
  t1.set_tool_number(1);

  tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.335);
  t2.set_cut_length(0.72);

  t2.set_shank_diameter(0.336);
  t2.set_shank_length(0.01);

  t2.set_holder_diameter(1.8);
  t2.set_holder_length(3.0);
  t2.set_tool_number(2);

  tool t3(0.5, 3.0, 2, HSS, FLAT_NOSE);
  t3.set_cut_diameter(0.5);
  t3.set_cut_length(0.7);

  t3.set_shank_diameter(0.7);
  t3.set_shank_length(0.5);

  t3.set_holder_diameter(1.8);
  t3.set_holder_length(3.0);
  t3.set_tool_number(3);
  
  vector<tool> tools{t1, t2, t3}; //, t3, t4};

  return fabrication_inputs(fixes, tools, workpiece_dims);
}

fabrication_inputs dice_inputs() {
  vice test_v = current_setup();
  vice test_vice = top_jaw_origin_vice(test_v);

  std::vector<plate_height> plates{0.48}; //0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(1.5, 1.58, 1.5, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(0.14);
  t1.set_cut_length(0.5);

  t1.set_shank_diameter(.375); //3.0 / 8.0);
  t1.set_shank_length(0.18);

  t1.set_holder_diameter(1.8);
  t1.set_holder_length(3.0);
  t1.set_tool_number(1);

  tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(0.335);
  t2.set_cut_length(0.72);

  t2.set_shank_diameter(0.336);
  t2.set_shank_length(0.01);

  t2.set_holder_diameter(1.8);
  t2.set_holder_length(3.0);
  t2.set_tool_number(2);

  tool t3(0.5, 3.0, 2, HSS, FLAT_NOSE);
  t3.set_cut_diameter(0.59);
  t3.set_cut_length(0.7);

  t3.set_shank_diameter(0.7);
  t3.set_shank_length(0.5);

  t3.set_holder_diameter(1.8);
  t3.set_holder_length(3.0);
  t3.set_tool_number(3);
  
  vector<tool> tools{t1, t2, t3}; //, t3, t4};

  return fabrication_inputs(fixes, tools, workpiece_dims);
}

fabrication_inputs part_1_29_inputs() {
  vice test_v = current_setup();
  vice test_vice = top_jaw_origin_vice(test_v);

  std::vector<plate_height> plates{0.48}; //0.1, 0.3, 0.7};
  fixtures fixes(test_vice, plates);

  workpiece workpiece_dims(1.5, 1.58, 1.5, ALUMINUM);

  tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  t1.set_cut_diameter(1.0 / 8.0);
  t1.set_cut_length(3.0 / 8.0);

  t1.set_shank_diameter(3.0 / 8.0); //.375); //3.0 / 8.0);
  t1.set_shank_length(0.1);

  t1.set_holder_diameter(1.8);
  t1.set_holder_length(3.0);
  t1.set_tool_number(1);

  tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
  t2.set_cut_diameter(3.0 / 8.0);
  t2.set_cut_length(0.75);

  t2.set_shank_diameter(3.0 / 8.0);
  t2.set_shank_length(0.1);

  t2.set_holder_diameter(1.8);
  t2.set_holder_length(3.0);
  t2.set_tool_number(2);

  tool t3(0.5, 3.0, 2, HSS, FLAT_NOSE);
  t3.set_cut_diameter(0.5);
  t3.set_cut_length(2.25);

  t3.set_shank_diameter(0.5);
  t3.set_shank_length(0.5);

  t3.set_holder_diameter(1.8);
  t3.set_holder_length(3.0);
  t3.set_tool_number(3);
  
  vector<tool> tools{t1, t2, t3};

  return fabrication_inputs(fixes, tools, workpiece_dims);
}

int main(int argc, char *argv[]) {

  DBG_ASSERT(argc == 2);

  string name = argv[1];
  cout << "File Name = " << name << endl;

  arena_allocator a;
  set_system_allocator(&a);

  fabrication_inputs fab_inputs = part_1_29_inputs(); //dice_inputs(); //octagon_with_holes_short_inputs(); //current_fab_inputs(workpiece(1.75, 1.75, 2.5, ALUMINUM)); //octagon_with_holes_short_inputs(); //part_1_2_inputs();
  
  triangular_mesh mesh =
    parse_stl(name, 0.0001);

  //vtk_debug_mesh(mesh);


  // NOTE: Used for part 1 (2)
  // double scale_factor = 0.45;

  double part_1_2_scale_factor = 0.45;
  double octagon_with_holes_short_scale_factor = 1.0; //0.15; //0.45;
  double japan_contour_scale_factor = 0.4;
  double dice_scale_factor = 0.7;
  double part_1_29_scale_factor = 0.2;
  auto scale_func = [part_1_29_scale_factor](const point p) {
    return part_1_29_scale_factor*p;
  };

  mesh =
    mesh.apply_to_vertices(scale_func);

  box b = mesh.bounding_box();

  cout << "BOX" << endl;
  cout << b << endl;
  cout << "X len = " << b.x_len() << endl;
  cout << "Y len = " << b.y_len() << endl;
  cout << "Z len = " << b.z_len() << endl;

  vtk_debug_mesh(mesh);

  fabrication_plan p =
    make_fabrication_plan(mesh, fab_inputs);

  double rapid_feed = 24.0;
  fab_plan_timing_info total_time;

  cout << "Number of steps = " << p.steps().size() << endl;
  for (auto& step : p.steps()) {
    cout << "STEP" << endl;
    auto step_time = make_timing_info(step, rapid_feed);

    print_time_info(cout, step_time);

    increment(total_time, step_time);
  }

  cout << "TOTAL Time Estimate" << endl;
  print_time_info(cout, total_time);

  cout << "Programs" << endl;

  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::showpoint);

  for (auto& step : p.steps()) {
    // point zero_pos = gui_select_part_zero(step);
    // cout << "Part zero position = " << zero_pos << endl;

    // fabrication_setup shifted_setup = shift(-1*zero_pos, step);
    // print_setup_info(shifted_setup);
    // visual_debug(shifted_setup);

    print_setup_info(step);
    cout << "Program for setup" << endl;
    auto program = step.gcode_for_toolpaths(emco_f1_code_no_TLC);
    cout << program.name << endl;
    cout << program.blocks << endl;

    visual_debug(step);

  }

  return EXIT_SUCCESS;
}

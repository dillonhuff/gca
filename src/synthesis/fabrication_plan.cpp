#include "synthesis/fabrication_plan.h"

namespace gca {

  std::vector<tool> current_tools() {

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.14);
    t1.set_cut_length(0.5);

    t1.set_shank_diameter(.375);
    t1.set_shank_length(0.18);

    t1.set_holder_diameter(1.8);
    t1.set_holder_length(3.0);

    tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.335);
    t2.set_cut_length(0.72);

    t2.set_shank_diameter(0.336);
    t2.set_shank_length(0.01);

    t2.set_holder_diameter(1.8);
    t2.set_holder_length(3.0);

    tool t3(0.125, 3.0, 4, HSS, FLAT_NOSE);
    t3.set_cut_diameter(0.125);
    t3.set_cut_length(1.0);

    t3.set_shank_diameter(.375);
    t3.set_shank_length(0.18);

    t3.set_holder_diameter(1.8);
    t3.set_holder_length(3.0);

    tool t4{1.0, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(1.0);
    t4.set_cut_length(1.0);

    t4.set_shank_diameter(1.1);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);

    tool t5(0.0625, 3.0, 4, HSS, FLAT_NOSE);
    t5.set_cut_diameter(0.0625);
    t5.set_cut_length(0.4);

    t5.set_shank_diameter(.375);
    t5.set_shank_length(0.18);

    t5.set_holder_diameter(1.8);
    t5.set_holder_length(3.0);

    tool t6{1.0 / 8.0, 3.94, 4, HSS, BALL_NOSE};
    t6.set_cut_diameter(1.0 / 4.0);
    t6.set_cut_length(1.25);

    t6.set_shank_diameter(0.5);
    t6.set_shank_length(0.05);

    t6.set_holder_diameter(2.5);
    t6.set_holder_length(3.5);
    t6.set_tool_number(4);
    
    vector<tool> tools{t1, t2, t3, t4, t5, t6};

    return tools;
  }

  fabrication_inputs current_fab_inputs(const workpiece& workpiece_dims) { 
    vice test_v = current_setup();
    vice test_vice = top_jaw_origin_vice(test_v);

    std::vector<plate_height> plates{0.48, 0.625, 0.7};
    fixtures fixes(test_vice, plates);

    auto tools = current_tools();

    return fabrication_inputs(fixes, tools, workpiece_dims);
  }

  fabrication_inputs extended_fab_inputs() {
    vice test_v =
      custom_jaw_vice_with_clamp_dir(4.0, 1.0, 8.0, point(0.0, 0.0, 0.0), point(1, 0, 0));
    vice test_vice = top_jaw_origin_vice(test_v);
    
    std::vector<plate_height> plates{0.1, 0.3, 0.7};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(3.5, 3.5, 3.8, ALUMINUM);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.5);
    t2.set_cut_length(0.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    tool t3{1.0 / 8.0, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(1.0 / 8.0);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);

    tool t4{1.5, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(1.5);
    t4.set_cut_length(2.2);

    t4.set_shank_diameter(0.5);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);

    tool t5{1.0 / 8.0, 3.94, 4, HSS, BALL_NOSE};
    t5.set_cut_diameter(1.0 / 4.0);
    t5.set_cut_length(1.25);

    t5.set_shank_diameter(0.5);
    t5.set_shank_length(0.05);

    t5.set_holder_diameter(2.5);
    t5.set_holder_length(3.5);
    t5.set_tool_number(4);
    
    vector<tool> tools{t1, t2, t3, t4, t5};

    return fabrication_inputs(fixes, tools, workpiece_dims);
  }

}

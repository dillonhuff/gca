#include "backend/feedrate_optimization.h"

namespace gca {

  class region bounding_region(const flat_region& mregion) {
    box b = bounding_box(mregion.safe_area);
    double x_len = b.x_max - b.x_min;
    double y_len = b.y_max - b.y_min;
    double z_len = mregion.height();

    class region r(x_len, y_len, z_len, 0.01);
    r.set_machine_x_offset(-b.x_min);
    r.set_machine_y_offset(-b.y_min);
    r.set_machine_z_offset(-mregion.end_depth); //-b.z_min);

    // TODO: Find better way to express the safe z value in the
    // machine coordinate system
    // point safe_machine_point(0, 0, mregion.start_depth()); //mregion.height());
    // point safe_region_point = r.machine_coords_to_region_coords(safe_machine_point);

    auto poly_2d = to_boost_multipoly_2(mregion.machine_area);

    for (int i = 0; i < r.num_x_elems; i++) {
      for (int j = 0; j < r.num_y_elems; j++) {
	double r_x = r.resolution*i;
	double r_y = r.resolution*j;

	point dummy(r_x, r_y, 0.0);
	point converted = r.region_coords_to_machine_coords(dummy);

	bg::model::d2::point_xy<double> conv_pt(converted.x, converted.y);

	if (bg::within(conv_pt, poly_2d)) {
	  r.set_column_height(i, j, mregion.height()); //safe_region_point.z);
	} else {
	  r.set_column_height(i, j, 0.0);
	}
      }
    }

    return r;
  }

  void optimize_feedrates_by_MRR_simulation(class region& sim_region,
					    toolpath& tp,
					    const double machine_hp,
					    const double material_unit_hp) {

    cylindrical_bit current_tool(tp.t.cut_diameter());

    for (auto cuts : tp.cuts_without_safe_moves()) {

      for (auto c : cuts) {
	double cut_volume = update_cut(*c, sim_region, current_tool);
	double cut_mrr = cut_volume / cut_execution_time_minutes(c);
	double cut_power = material_unit_hp*cut_mrr;

	cout << "Cut required power = " << cut_power << " hp" << endl;

	if (cut_power >= machine_hp) {

	  if (c->is_safe_move()) {
	    cout << "Cut that requires large power is a safe move?" << endl;
	    DBG_ASSERT(false);
	  }

	  double scaled_down_power = 0.8*machine_hp;

	  double scaled_down_feed =
	    (scaled_down_power*(c->length())) / (material_unit_hp * cut_volume);

	  cout << "Scaling down feed to " << scaled_down_feed << endl;

	  c->set_feedrate(lit::make(scaled_down_feed));
	}

      }

    }

  }
  
  void optimize_feedrates_by_MRR_simulation(const flat_region& r,
					    std::vector<toolpath>& toolpaths,
					    const double machine_hp,
					    const double material_unit_hp) {

    class region sim_region = bounding_region(r);

    for (auto& tp : toolpaths) {
      optimize_feedrates_by_MRR_simulation(sim_region,
					   tp,
					   machine_hp,
					   material_unit_hp);
    }

  }

}

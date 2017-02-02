#include "backend/gcode_generation.h"
#include "synthesis/mesh_to_gcode.h"
#include "backend/toolpath_generation.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/algorithm.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w) {
    out << "WORKPIECE" << endl;
    out << w.sides[0] << endl;
    out << w.sides[1] << endl;
    out << w.sides[2] << endl;
    return out;
  }

  std::vector<toolpath> cut_secured_mesh(vector<pocket>& pockets,
					 const std::vector<tool>& tools,
					 const material& stock_material) {
    DBG_ASSERT(tools.size() > 0);

    vector<toolpath> toolpaths =
      mill_pockets(pockets, tools, stock_material);

    return toolpaths;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const fixtures& f,
					   const vector<tool>& tools,
					   const workpiece w) {
    fabrication_plan plan = make_fabrication_plan(part_mesh, f, tools, {w});
    vector<gcode_program> progs;
    for (auto f : plan.steps()) {
      progs.push_back(f.gcode_for_toolpaths(emco_f1_code_G10_TLC));
    }
    return progs;
  }

  fabrication_plan make_fabrication_plan(const triangular_mesh& part_mesh,
					 const fabrication_inputs& inputs) {
    return make_fabrication_plan(part_mesh,
				 inputs.f,
				 inputs.tools,
				 {inputs.w});
  }

  fabrication_plan
  fabrication_plan_for_fixture_plan(const fixture_plan& plan,
				     const triangular_mesh& part_mesh,
				     const std::vector<tool>& tools,
				     const workpiece& w) {
    vector<fabrication_setup> setups;
    for (auto setup : plan.fixtures()) {
      auto toolpaths =
	cut_secured_mesh(setup.pockets, tools, w.stock_material);
      setups.push_back(fabrication_setup(setup.arrangement(), setup.fix.v, toolpaths));
    }

    return fabrication_plan(&part_mesh, setups, plan.custom_fixtures());
  }

  fabrication_plan make_fabrication_plan(const triangular_mesh& part_mesh,
					 const fixtures& f,
					 const vector<tool>& tools,
					 const std::vector<workpiece>& wps) {
    fixture_plan plan = make_fixture_plan(part_mesh, f, tools, wps);

    return fabrication_plan_for_fixture_plan(plan, part_mesh, tools, plan.stock());
  }

  void print_programs(const fabrication_plan& fix_plan) {
    cout << "Programs" << endl;

    cout.setf(ios::fixed, ios::floatfield);
    cout.setf(ios::showpoint);

    for (auto f : fix_plan.steps()) {
      auto program = f.gcode_for_toolpaths(emco_f1_code);
      cout << program.name << endl;
      cout << program.blocks << endl;
    }

  }

  void print_programs_no_TLC(const fabrication_plan& fix_plan) {
    cout << "Programs" << endl;

    cout.setf(ios::fixed, ios::floatfield);
    cout.setf(ios::showpoint);

    for (auto f : fix_plan.steps()) {
      auto program = f.gcode_for_toolpaths(emco_f1_code_no_TLC);
      cout << program.name << endl;
      cout << program.blocks << endl;
    }

  }

  void print_programs_wells_no_TLC(const fabrication_plan& fix_plan) {
    cout << "Programs" << endl;

    cout.setf(ios::fixed, ios::floatfield);
    cout.setf(ios::showpoint);

    for (auto f : fix_plan.steps()) {
      auto program = f.gcode_for_toolpaths(wells_code_no_TLC);
      cout << program.name << endl;
      cout << program.blocks << endl;
    }

  }
  
}

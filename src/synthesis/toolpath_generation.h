#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/box.h"
#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/cut.h"
#include "synthesis/tool.h"

namespace gca {

  struct pocket {
  private:
    oriented_polygon boundary;
    std::vector<oriented_polygon> holes;

    double start_depth;
    std::vector<index_t> base_inds;
    const triangular_mesh& mesh;

  public:
    pocket(const oriented_polygon& boundary,
	   std::vector<oriented_polygon>& holesp,
	   double start_depthp,
	   const std::vector<index_t>& basep,
	   const triangular_mesh& p_mesh) :
      boundary(boundary),
      holes(holesp),
      start_depth(start_depthp),
      base_inds(basep),
      mesh(p_mesh) {
      assert(base_inds.size() > 0);
    }

    const std::vector<index_t>& base_face_indexes() const
    { return base_inds; }

    const triangular_mesh& base_mesh() const
    { return mesh; }
    
    inline const vector<oriented_polygon>& get_holes() const
    { return holes; }

    inline const oriented_polygon& get_boundary() const
    { return boundary; }

    inline double get_start_depth() const { return start_depth; }

    inline double get_end_depth() const {
      vector<point> base_points;
      for (auto i : base_inds) {
	triangle_t t = mesh.triangle_vertices(i);
	base_points.push_back(mesh.vertex(t.v[0]));
	base_points.push_back(mesh.vertex(t.v[1]));
	base_points.push_back(mesh.vertex(t.v[2]));
      }
      return min_distance_along(base_points, point(0, 0, 1));
    }

    inline std::vector<triangle> base() const {
      std::vector<triangle> base_tris;
      for (auto i : base_inds) {
	base_tris.push_back(mesh.face_triangle(i));
      }
      return base_tris;
    }

    bool above_base(const point p) {
      for (auto i : base_inds) {
	auto t = mesh.face_triangle(i);
	if (in_projection(t, p) && below(t, p)) { return false; }
      }
      return true;
    }

    box bounding_box() const {
      return mesh.bounding_box();
    }
  };

  std::vector<polyline> deepen_polyline(const std::vector<double>& depths,
					const polyline& p);

  std::vector<double> cut_depths(double start_depth,
				 double end_depth,
				 double cut_depth);

  std::vector<polyline> tile_vertical(const std::vector<polyline>& ps,
				      double start_depth,
				      double end_depth,
				      double cut_depth);

  std::vector<polyline> repeated_offsets(const polyline& p,
					 int num_repeats,
					 offset_dir d,
					 double inc);

  std::vector<polyline> rough_pocket(const pocket& pocket,
				     const tool& t,
				     double cut_depth);

  std::vector<polyline> rough_pockets(const std::vector<pocket>& pockets,
				      const tool& t,
				      double cut_depth);

  std::vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					     const tool& t,
					     double cut_depth);

  std::vector<cut*> polyline_cuts(const polyline& p);

  polyline compress_lines(const polyline& p, double tolerance);

  std::vector<polyline> rough_box(const box b,
				  double tool_radius,
				  double cut_depth);

  std::vector<polyline> drop_sample(const triangular_mesh& mesh,
				    const tool& tool);

  std::vector<point> drop_points_onto(const std::vector<point>& pts_z,
				      const std::vector<index_t>& faces,
				      const triangular_mesh& mesh,
				      const tool& tool);

  std::vector<block> emco_f1_code(const std::vector<polyline>& pocket_lines,
				  const double safe_height);

}

#endif

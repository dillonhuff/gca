#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include <memory>

#include "geometry/box.h"
#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "gcode/cut.h"
#include "synthesis/tool.h"

namespace gca {

  struct toolpath {
    tool t;
    std::vector<polyline> lines;

    toolpath(const tool& p_t,
	     const std::vector<polyline>& p_lines)
      : t(p_t), lines(p_lines) {}
  };

  // TODO: Move to triangular mesh?
  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh);

  class pocket {
  public:
    template<typename T>
    pocket(T x) : self_(new model<T>(move(x))) {}

    pocket(const pocket& x) : self_(x.self_->copy_()) {}

    pocket(pocket&&) noexcept = default;

    pocket& operator=(const pocket& x)
    { pocket tmp(x); *this = move(tmp); return *this; }

    pocket& operator=(pocket&&) noexcept = default;

    tool select_tool(const std::vector<tool>& tools) const
    { return self_->select_tool(tools); }
    const vector<oriented_polygon>& get_holes() const
    { return self_->get_holes(); }
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const
    { return self_->toolpath_lines(t, cut_depth); }
    bool above_base(const point p) const
    { return self_->above_base(p); }
    double get_end_depth() const
    { return self_->get_end_depth(); }
    double get_start_depth() const
    { return self_->get_start_depth(); }

  private:
    struct concept_t {
      virtual ~concept_t() = default;
      virtual tool select_tool(const std::vector<tool>& tools) const = 0;
      virtual const vector<oriented_polygon>& get_holes() const = 0;
      virtual double get_end_depth() const = 0;
      virtual double get_start_depth() const = 0;
      virtual bool above_base(const point p) const = 0;
      virtual std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const = 0;
      virtual concept_t* copy_() const = 0;
    };

    template<typename T>
    struct model : concept_t {
      model(T x) : data_(move(x)) {}
      tool select_tool(const std::vector<tool>& tools) const
      { return data_.select_tool(tools); }
      virtual concept_t* copy_() const { return new model<T>(*this); }
      const vector<oriented_polygon>& get_holes() const
      { return data_.get_holes(); }
      virtual std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const
      { return data_.toolpath_lines(t, cut_depth); }
      bool above_base(const point p) const { return data_.above_base(p); }
      double get_end_depth() const { return data_.get_end_depth(); }
      double get_start_depth() const { return data_.get_start_depth(); }
      T data_;
    };
  
    unique_ptr<concept_t> self_;
  };

  struct freeform_pocket {
  private:
    oriented_polygon boundary;
    std::vector<oriented_polygon> holes;

    double start_depth;
    std::vector<index_t> base_inds;
    const triangular_mesh* mesh;

  public:
    freeform_pocket(double start_depthp,
	   const std::vector<index_t>& basep,
	   const triangular_mesh* p_mesh) :
      start_depth(start_depthp),
      base_inds(basep),
      mesh(p_mesh)
    {
      assert(base_inds.size() > 0);
      auto bounds = mesh_bounds(base_inds, base_mesh());
      assert(bounds.size() > 0);
      boundary = extract_boundary(bounds);
      holes = bounds;
    }

    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;

    const std::vector<index_t>& base_face_indexes() const
    { return base_inds; }

    const triangular_mesh& base_mesh() const
    { return *mesh; }
    
    inline const vector<oriented_polygon>& get_holes() const
    { return holes; }

    inline const oriented_polygon& get_boundary() const
    { return boundary; }

    inline double get_start_depth() const { return start_depth; }

    // TODO: Optimize to use vertex list
    inline double get_end_depth() const {
      vector<point> base_points;
      for (auto i : base_inds) {
	triangle_t t = mesh->triangle_vertices(i);
	base_points.push_back(mesh->vertex(t.v[0]));
	base_points.push_back(mesh->vertex(t.v[1]));
	base_points.push_back(mesh->vertex(t.v[2]));
      }
      return min_distance_along(base_points, point(0, 0, 1));
    }

    inline std::vector<triangle> base() const {
      std::vector<triangle> base_tris;
      for (auto i : base_inds) {
	base_tris.push_back(mesh->face_triangle(i));
      }
      return base_tris;
    }

    bool above_base(const point p) const {
      for (auto i : base_inds) {
	auto t = mesh->face_triangle(i);
	if (in_projection(t, p) && below(t, p)) { return false; }
      }
      return true;
    }

    box bounding_box() const {
      return mesh->bounding_box();
    }
  };

  class contour_pocket {
  protected:
    double start_depth;
    double end_depth;
    oriented_polygon interior;
    oriented_polygon exterior;

  public:
    contour_pocket(const double p_start_depth,
		   const double p_end_depth,
		   const oriented_polygon& p_i,
		   const oriented_polygon& p_e)
      : start_depth(p_start_depth), end_depth(p_end_depth),
	interior(p_i), exterior(p_e) {}
    
    const vector<oriented_polygon>& get_holes() const
    { assert(false); }

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;
  };
  
  class face_pocket {
  protected:
    double start_depth;
    double end_depth;
    oriented_polygon base;

  public:
    face_pocket(const double p_start_depth,
		const double p_end_depth,
		const oriented_polygon& p_base)
      : start_depth(p_start_depth), end_depth(p_end_depth), base(p_base) {}

    const vector<oriented_polygon>& get_holes() const
    { assert(false); }

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    tool select_tool(const std::vector<tool>& tools) const;

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;
  };

  pocket box_pocket(const box b);

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

  // std::vector<polyline> pocket_2P5D_interior(const pocket& pocket,
  // 					     const tool& t,
  // 					     double cut_depth);

  std::vector<cut*> polyline_cuts(const polyline& p);

  polyline compress_lines(const polyline& p, double tolerance);

  std::vector<polyline> drop_sample(const triangular_mesh& mesh,
				    const tool& tool);

  std::vector<point> drop_points_onto(const std::vector<point>& pts_z,
				      const std::vector<index_t>& faces,
				      const triangular_mesh& mesh,
				      const tool& tool);

  std::vector<block> emco_f1_code(const toolpath& pocket_lines,
				  const double safe_height,
				  const material& stock_material);

  std::vector<point> drop_points_onto_max(const std::vector<point>& pts_z,
					  const std::vector<index_t>& faces,
					  const triangular_mesh& mesh,
					  const double max,
					  const tool& tool);
}

#endif

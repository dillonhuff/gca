#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/box.h"
#include "synthesis/operation.h"

namespace gca {

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
		    const triangular_mesh* p_mesh);

    pocket_name pocket_type() const { return FREEFORM_POCKET; }
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

    toolpath make_toolpath(const material& stock_material,
			   const double safe_z,
			   const std::vector<tool>& tools) const;
    
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

  struct flat_pocket {
  private:
    oriented_polygon boundary;
    std::vector<oriented_polygon> holes;

    double start_depth;
    double end_depth;

    std::vector<tool> possible_tools;

  public:
    flat_pocket(double start_depthp,
		const std::vector<index_t>& basep,
		const triangular_mesh* p_mesh,
		const std::vector<tool>& p_tools);

    flat_pocket(double p_start_depth,
		double p_end_depth,
		const oriented_polygon& p_boundary,
		const std::vector<tool>& p_tools);

    flat_pocket(double p_start_depth,
		double p_end_depth,
		const oriented_polygon& p_boundary,
		const std::vector<oriented_polygon>& p_holes,
		const std::vector<tool>& p_tools);

    polygon_3 base() const;

    std::vector<polyline>
    flat_level_with_holes(const tool& t) const;

    pocket_name pocket_type() const { return FLAT_POCKET; }
    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;

    inline const vector<oriented_polygon>& get_holes() const
    { return holes; }

    inline const oriented_polygon& get_boundary() const
    { return boundary; }

    toolpath make_toolpath(const material& stock_material,
			   const double safe_z,
			   const std::vector<tool>& tools) const;
    
    inline double get_start_depth() const { return start_depth; }

    inline double get_end_depth() const { return end_depth; }

    bool above_base(const point p) const
    { return p.z > get_end_depth(); }      

  };

  struct contour {
  private:
    polygon_3 bp;

    double start_depth;
    double end_depth;

    std::vector<tool> possible_tools;

  public:
    contour(double p_start_depth,
	    double p_end_depth,
	    const polygon_3& p_bp,
	    const std::vector<tool>& p_tools) :
      start_depth(p_start_depth),
      end_depth(p_end_depth),
      bp(p_bp),
      possible_tools(p_tools) {}

    const polygon_3& base() const { return bp; }

    vector<oriented_polygon> get_holes() const {
      vector<oriented_polygon> hs;
      for (auto h : base().holes()) {
	hs.push_back(oriented_polygon(base().normal(), h));
      }

      return hs;
    }

    oriented_polygon get_boundary() const {
      return oriented_polygon(base().normal(), base().vertices());
    }
    
    std::vector<polyline>
    flat_level_with_holes(const tool& t) const;

    pocket_name pocket_type() const { return CONTOUR; }
    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;

    toolpath make_toolpath(const material& stock_material,
			   const double safe_z,
			   const std::vector<tool>& tools) const;
    
    inline double get_start_depth() const { return start_depth; }

    inline double get_end_depth() const { return end_depth; }

    bool above_base(const point p) const
    { return p.z > get_end_depth(); }      

  };
  
  class face_pocket {
  protected:
    double start_depth;
    double end_depth;
    oriented_polygon base;
    std::vector<tool> possible_tools;

  public:
    face_pocket(const double p_start_depth,
		const double p_end_depth,
		const oriented_polygon& p_base,
		const std::vector<tool>& p_possible_tools)
      : start_depth(p_start_depth), end_depth(p_end_depth), base(p_base),
	possible_tools(p_possible_tools){}

    const vector<oriented_polygon>& get_holes() const
    { DBG_ASSERT(false); }

    pocket_name pocket_type() const { return FACE_POCKET; }    

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    toolpath make_toolpath(const material& stock_material,
			   const double safe_z,
			   const std::vector<tool>& tools) const;
    
    tool select_tool(const std::vector<tool>& tools) const;

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;
  };

  class trace_pocket {
  protected:
    double start_depth;
    double end_depth;
    oriented_polygon outline;

  public:
    trace_pocket(const double p_start_depth,
		const double p_end_depth,
		const oriented_polygon& p_outline)
      : start_depth(p_start_depth), end_depth(p_end_depth), outline(p_outline) {}

    const vector<oriented_polygon>& get_holes() const
    { DBG_ASSERT(false); }

    pocket_name pocket_type() const { return TRACE_POCKET; }    

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    toolpath make_toolpath(const material& stock_material,
			   const double safe_z,
			   const std::vector<tool>& tools) const;
    
    tool select_tool(const std::vector<tool>& tools) const;

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;
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

  polyline compress_lines(const polyline& p, double tolerance);

  std::vector<polyline> drop_sample(const triangular_mesh& mesh,
				    const tool& tool);

  std::vector<point> drop_points_onto(const std::vector<point>& pts_z,
				      const std::vector<index_t>& faces,
				      const triangular_mesh& mesh,
				      const tool& tool);

  std::vector<point> drop_points_onto_max(const std::vector<point>& pts_z,
					  const std::vector<index_t>& faces,
					  const triangular_mesh& mesh,
					  const double max,
					  const tool& tool);

  vector<toolpath> mill_pockets(vector<pocket>& pockets,
				const std::vector<tool>& tools,
				const material& stock_material);



}

#endif

#ifndef GCA_TOOLPATH_GENERATION_H
#define GCA_TOOLPATH_GENERATION_H

#include "geometry/box.h"
#include "backend/operation.h"

namespace gca {

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

    inline double volume() const
    { return area(base())*(get_start_depth() - get_end_depth()); }

    std::vector<polyline>
    flat_level_with_holes(const tool& t) const;

    pocket_name pocket_type() const { return FLAT_POCKET; }
    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;

    inline vector<polygon_3> get_holes() const {
      std::vector<polygon_3> h_polys;
      for (auto p : holes) {
	h_polys.push_back(build_clean_polygon_3(p.vertices()));
      }
      return h_polys;
    }

    inline polygon_3 get_boundary() const
    { return build_clean_polygon_3(boundary.vertices()); }

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const;

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z) const;
    
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

    inline vector<polygon_3> get_holes() const {
      std::vector<polygon_3> h_polys;
      for (auto p : bp.holes()) {
	h_polys.push_back(build_clean_polygon_3(p));
      }
      return h_polys;
    }

    inline double volume() const
    { return area(base())*(get_start_depth() - get_end_depth()); }

    inline polygon_3 get_boundary() const
    { return build_clean_polygon_3(bp.vertices()); }

    std::vector<polyline>
    flat_level_with_holes(const tool& t) const;

    pocket_name pocket_type() const { return CONTOUR; }
    tool select_tool(const std::vector<tool>& tools) const;
    std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const;

    std::vector<toolpath> make_finish_toolpaths(const material& stock_material,
						const double safe_z);
    
    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const;

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z) const;
    
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

    inline double volume() const {
      double ar = area(build_clean_polygon_3(base.vertices()));
      double height = get_start_depth() - get_end_depth();
      cout << "Pocket base area = " << ar << endl;
      cout << "Pocket height    = " << height << endl;
      return ar*height;
    }


    pocket_name pocket_type() const { return FACE_POCKET; }    

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const;

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z) const;

    toolpath make_finish_toolpath(const material& stock_material,
				  const double safe_z) const;
    
    tool select_tool(const std::vector<tool>& tools) const;

    std::vector<polyline>
    toolpath_lines(const tool& t, const double cut_depth) const;
  };

  class trace_pocket {
  protected:
    double start_depth;
    double end_depth;
    oriented_polygon outline;
    std::vector<tool> possible_tools;

  public:
    trace_pocket(const double p_start_depth,
		 const double p_end_depth,
		 const oriented_polygon& p_outline,
		 const std::vector<tool>& p_possible_tools)
      : start_depth(p_start_depth),
	end_depth(p_end_depth),
	outline(p_outline),
	possible_tools(p_possible_tools) {}

    const vector<oriented_polygon>& get_holes() const
    { DBG_ASSERT(false); }

    pocket_name pocket_type() const { return TRACE_POCKET; }    

    double get_end_depth() const
    { return end_depth; }
    double get_start_depth() const
    { return start_depth; }
    bool above_base(const point p) const
    { return p.z > get_end_depth(); }

    double volume() const { return 0.0; }

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z,
					 const std::vector<tool>& tools) const;

    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z) const;
    
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

  vector<toolpath> mill_pockets(const vector<pocket>& pockets,
				const material& stock_material);


  class flat_region {
  public:

    polygon_3 safe_area;
    vector<polygon_3> machine_area;

    double start_depth, end_depth;
    material stock_material;

    
    flat_region(const polygon_3& p_safe_area,
		const polygon_3& p_machine_area,
		const double p_start_depth,
		const double p_end_depth,
		const material p_stock_material) :
      safe_area(p_safe_area),
      machine_area{p_machine_area},
      start_depth(p_start_depth),
      end_depth(p_end_depth),
      stock_material(p_stock_material) {
      
    }

    flat_region(const polygon_3& p_safe_area,
		const std::vector<polygon_3>& p_machine_area,
		const double p_start_depth,
		const double p_end_depth,
		const material p_stock_material) :
      safe_area(p_safe_area),
      machine_area(p_machine_area),
      start_depth(p_start_depth),
      end_depth(p_end_depth),
      stock_material(p_stock_material) {
      
    }

    double height() const { return start_depth - end_depth; }

    flat_region shift(const point p) const {
      return flat_region(gca::shift(p, safe_area),
			 gca::shift(p, machine_area),
			 start_depth + p.z,
			 end_depth + p.z,
			 stock_material);
    }

  };



  std::vector<toolpath>
  machine_flat_region(const flat_region& r,
		      const double safe_z,
		      const std::vector<tool>& tools);

  std::vector<toolpath>
  machine_flat_region_with_contours(const flat_region& r,
				    const double safe_z,
				    const std::vector<tool>& tools);

  std::vector<toolpath>
  finish_flat_region_with_contours(const flat_region& r,
				   const double safe_z,
				   const std::vector<tool>& all_tools);
  
  boost_linestring_2 to_boost_linestring(const polyline& pl);

  std::vector<polyline>
  zig_lines(const polygon_3& bound,
	    const std::vector<polygon_3>& holes,
	    const tool& t);

  std::vector<polyline>
  clip_lines(const std::vector<polyline>& lines,
	     const polygon_3& bound,
	     const std::vector<polygon_3>& hole_polys,
	     const tool& t);

  std::vector<polyline>
  zig_lines(const polygon_3& bound,
	    const std::vector<polygon_3>& holes,
	    const double stepover,
	    const tool& t);
  
}

#endif

#ifndef GCA_OPERATION_H
#define GCA_OPERATION_H

#include <iostream>
#include <memory>

#include "backend/operation_name.h"
#include "geometry/polygon.h"
#include "geometry/polygon_3.h"
#include "geometry/polyline.h"
#include "geometry/triangle.h"
#include "geometry/triangular_mesh.h"
#include "backend/material.h"
#include "backend/toolpath.h"
#include "utils/check.h"


namespace gca {

  class pocket {
  public:
    template<typename T>
    pocket(T x) : self_(new model<T>(move(x))) {}

    pocket(const pocket& x) : self_(x.self_->copy_()) {}

    pocket(pocket&&) noexcept = default;

    pocket& operator=(const pocket& x)
    { pocket tmp(x); *this = move(tmp); return *this; }

    pocket& operator=(pocket&&) noexcept = default;

    pocket_name pocket_type() const { return self_->pocket_type(); }

    // std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const
    // { return self_->toolpath_lines(t, cut_depth); }

    double get_end_depth() const
    { return self_->get_end_depth(); }

    double get_start_depth() const
    { return self_->get_start_depth(); }

    // std::vector<toolpath> make_toolpaths(const material& stock_material,
    // 					 const double safe_z,
    // 					 const std::vector<tool>& tools) const
    // { return self_->make_toolpaths(stock_material, safe_z, tools); }
    
    std::vector<toolpath> make_toolpaths(const material& stock_material,
					 const double safe_z) const
    { return self_->make_toolpaths(stock_material, safe_z); }

    double volume() const
    { return self_->volume(); }

  private:
    struct concept_t {
      virtual ~concept_t() = default;

      virtual pocket_name pocket_type() const = 0;
      virtual std::vector<toolpath>
      make_toolpaths(const material& stock_material,
		     const double safe_z) const = 0;

      virtual std::vector<toolpath>
      make_toolpaths(const material& stock_material,
		     const double safe_z,
		     const std::vector<tool>& tools) const = 0;
      
      virtual double get_end_depth() const = 0;
      virtual double get_start_depth() const = 0;
      virtual double volume() const = 0;

      // virtual std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const = 0;
      virtual concept_t* copy_() const = 0;
    };

    template<typename T>
    struct model : concept_t {
      model(T x) : data_(move(x)) {}
      pocket_name pocket_type() const { return data_.pocket_type(); }

      virtual concept_t* copy_() const { return new model<T>(*this); }

      // virtual std::vector<polyline> toolpath_lines(const tool& t, const double cut_depth) const
      // { return data_.toolpath_lines(t, cut_depth); }

      double get_end_depth() const { return data_.get_end_depth(); }
      double get_start_depth() const { return data_.get_start_depth(); }

      double volume() const { return data_.volume(); }

      virtual std::vector<toolpath>
      make_toolpaths(const material& stock_material,
		     const double safe_z,
		     const std::vector<tool>& tools) const
      { return data_.make_toolpaths(stock_material, safe_z, tools); }

      virtual std::vector<toolpath>
      make_toolpaths(const material& stock_material,
		     const double safe_z) const
      { return data_.make_toolpaths(stock_material, safe_z); }
      
      T data_;
    };
  
    unique_ptr<concept_t> self_;
  };


}

#endif

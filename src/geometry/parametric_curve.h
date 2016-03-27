#ifndef GCA_PARAMETRIC_CURVE_H
#define GCA_PARAMETRIC_CURVE_H

namespace gca {

  class parametric_curve {
  private:
    struct concept_t {
      virtual ~concept_t() = default;
      virtual concept_t* copy() const = 0;
      virtual point value(double t) const = 0;
    };

    template<typename T>
    struct model : concept_t {
      T data;
      model(T x) : data(move(x)) {}
      virtual concept_t* copy() const { return new model<T>(*this); }
      virtual point value(double t) const { return data.value(t); }
    };
  
    unique_ptr<concept_t> self;

  public:
    template<typename T>
    parametric_curve(T x) : self(new model<T>(move(x))) {}
    parametric_curve(const parametric_curve& x) : self(x.self->copy()) {}
    parametric_curve(parametric_curve&&) noexcept = default;

    parametric_curve& operator=(const parametric_curve& x)
    { parametric_curve tmp(x); *this = move(tmp); return *this; }

    parametric_curve& operator=(parametric_curve&&) noexcept = default;
    
    inline point value(double t) const
    { return self->value(t); }
  };
}

#endif
